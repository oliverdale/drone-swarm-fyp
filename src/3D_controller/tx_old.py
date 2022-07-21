"""
filename: tx.py
author: Reka Norman
date: 16th October 2020
description: The main program which runs on the Tx UAV.
"""

import time
import zmq
import sys
import matplotlib.pyplot as plt
import argparse
import traceback

from zmq.sugar.context import T
import rospy
from zmq.sugar.constants import PLAIN_PASSWORD

from common import *
import multilateration
from gps import GPSCoord
from packets import RxUpdate, TxUpdate
from mavros_offboard_posctl import MultiMavrosOffboardPosctl
from swarming_logic import Swarming
from update_store import UpdateStore
import logger
# Logging Stuff
from paramiko import SSHClient
from paramiko.client import AutoAddPolicy

flight_parameters = get_parameters()

# Size of the plot - measured from the Tx start position to each edge.
PLOT_RANGE_M = 40

# Number of points on the plot at any one time. Points older than this are
# removed to make the plot clearer.
NUM_POINTS = 10

# Timeout period for receiving updates from Rxs.
TIMEOUT_S = 3


class TimeoutException(Exception):
    """ Raised when a timeout occurs waiting for updates from the Rxs. """
    pass


class TransmitterUAV:
    def __init__(self, context, should_plot, mavros_controller=None):

        self.context = context
        self.should_plot = should_plot
        self.mavros_controller = mavros_controller
        self.state = ON_STATE

        items_to_log = [
            'states_list',
            'cart_target_filt',
            'gps_target_filt',
            'gps_target_real',
            'cart_tx_filt',
            'gps_tx_filt',
            'gps_tx_real',
            'multilateration_error',
            'mode']
        self.logger = logger.Logger(0, items_to_log)

        self.flight_parameters = get_parameters()

        self.tx_ip = flight_parameters['0']['IP_ADDRESS']
        self.tx_recieve_port = flight_parameters['COMMON']['RECEIVE_PORT']
        self.tx_send_port = flight_parameters['COMMON']['SEND_PORT']
        self.tx_startup_port = flight_parameters['COMMON']['STARTUP_PORT']

        self.target_ip = flight_parameters['COMMON']['TARGET_POS_IP_ADDRESS']
        self.target_port = flight_parameters['COMMON']['TARGET_POS_PORT']

        self.controller_ip = flight_parameters['COMMON']['CONTROLLER_IP_ADDRESS']
        self.controller_port = flight_parameters['COMMON']['CONTROLLER_PORT']

        # Store all the updates received from the Rxs.
        self.updates = UpdateStore()

        self.receiver = None
        self.sender = None
        self.controller = None
        self.startup  = None

        self.est_target_pos = None
        self.act_target_pos = None
        self.pos_filter = None
        self.multilateration_error = None
        self.target_position_GPS = None # When this is not none the tx has its first target value


    def close_sockets(self):
        """
        Close the sending and recieving sockets when the TX terminates
        """
        if self.receiver is not None:
            self.receiver.close()
        if self.sender is not None:
            self.sender.close()
        if self.controller is not None:
            self.controller.close()
        if self.startup is not None:
            self.startup.close()


    def connect(self):
        success = True

        try:
            # PULL socket for receiving updates from the Rxs.
            self.receiver = self.context.socket(zmq.PULL)
            self.receiver.bind("tcp://*:{}".format(self.tx_recieve_port))
            # Poller required to set a timeout on receiving from the receiver socket.
            self.poller = zmq.Poller()
            self.poller.register(self.receiver, zmq.POLLIN)
        except Exception as e:
            rospy.logwarn("TX: Could not setup reciever connection")
            rospy.logfatal(traceback.format_exc())
            success = False

        try:
            # PUB socket for sending target position estimates to the Rxs.
            self.sender = self.context.socket(zmq.PUB)
            self.sender.bind("tcp://*:{}".format(self.tx_send_port))
        except Exception as e:
            rospy.logwarn("TX: Could not setup sender connection")
            rospy.logfatal(traceback.format_exc())
            success = False

        try:
            self.controller = self.context.socket(zmq.ROUTER)
            self.controller.connect("tcp://{}:{}".format(self.controller_ip, self.controller_port))
        except Exception as e:
            rospy.logwarn("TX: Could not setup controller connection")
            rospy.logfatal(traceback.format_exc())
            success = False
            
        try:
            self.startup = self.context.socket(zmq.ROUTER)
            self.startup.bind("tcp://*:{}".format(self.tx_startup_port))
        except Exception as e:
            rospy.logwarn("TX: Could not setup startup connection")
            rospy.logfatal(traceback.format_exc())
            success = False
        
        if not success:
            self.close_sockets()

        return success

    def get_first_pos(self):
        """
        Connets to the target server and gets the first target position. It sets self.target_position_GPS and
        self.tx_desired_coords to DEF_ALTITUDE above it. If this is successful it returns True. If target_position_GPS
        is already set it returns True and does not get a new value.
        """
        success = True
        # Initialise the variable storing the Tx UAV's current position.
        if self.target_position_GPS is None:
                socket = self.context.socket(zmq.SUB)
                socket.connect("tcp://{}:{}".format(self.target_ip, self.target_port))
                socket.setsockopt(zmq.SUBSCRIBE, b"")
                rospy.loginfo("TX: Receiving start position from target position server...")
                self.target_position_GPS = GPSCoord(*struct.unpack("ddd", socket.recv()))
                socket.close()

                self.tx_desired_coords = update_loc(self.target_position_GPS, TX_ID)
                
        assert (self.tx_desired_coords is not None)

        return success


    def get_tx_GPS(self):
        """ Returns the current position of the Tx drone.
        """
        return self.mavros_controller.get_current_position()


    def receive_updates(self):
        """ Repeatedly receive and store updates from the Rxs, returning True if
        a new full group of readings is ready. Raises a TimeoutException if
        this doesn't happen before the timeout_time
        """

        sockets = dict(self.poller.poll(timeout=100))
        if self.receiver in sockets:
            message = self.receiver.recv(flags=zmq.NOBLOCK)
            
            update = RxUpdate.from_bytes(message)

            # If this is the first time through the loop set these so the drone knows where to go
            if self.target_position_GPS is None:
                self.target_position_GPS = update.target_coords
                self.tx_desired_coords = update_loc(self.target_position_GPS, TX_ID)

            rospy.logwarn("TX: {}".format(str(update)))

            if self.updates.store(update):
                return True
        else:
            pass
            # # Print error message including the the time since an update
            # # was received from each Rx, to help diagnose the timeout.
            # message = "ERROR: Timeout occurred. Times since last update:\n"
            # times_since_update = [
            #     time.time() - last_time
            #     for last_time in self.updates.get_last_update_times()
            # ]
            # for rx_id in range(1, NUM_RXS + 1):
            #     message += "Rx {}: {:.3f} s\n".format(
            #         rx_id, times_since_update[rx_id - 1])
            # raise TimeoutException(message)

        return False

    def perform_multilateration(self, prev_est):
        """ 
        Returns the target position estimated by the multilateration module.
        """
        rx_positions = self.updates.get_rx_positions()
        ranges = self.updates.get_ranges()
        drones = [self.get_tx_GPS()] + rx_positions
        est_GPS_target_pos = self.pos_filter.est_target_pos(drones, ranges, 1)
        
        return est_GPS_target_pos

    def swarming_checks(self, tx_swarm):
        """ 
        Returns the desired centre position of the formation. 
        If formation is fine, output the target position, 
        otherwise outputs the averaged centre of the formation. 
        Discards unreasonable target GPS values.
        """
        drone_positions = [self.get_tx_GPS()
                           ] + self.updates.get_rx_positions()

        tx_swarm.update_drones(drone_positions)
        tx_swarm.swarming_checks()

    def swarming_destination(self, tx_swarm):
        tx_swarm.update_target(self.target_position_GPS)
        tx_swarm.update_swarming_destination()
        return tx_swarm.dest_coord

    def plot_positions(self):
        """ Plot the current positions of the Tx and the Rxs, and the actual
        target position. """
        colors = ['k', 'b', 'g', 'r', 'm']
        positions = [self.get_tx_GPS()] + self.updates.get_rx_positions()

        points = []
        for i in range(len(positions)):
            points += plt.plot(positions[i].long, positions[i].lat,
                               'x' + colors[i])

        # Get the actual target coords (sent from the Rxs for plotting).
        actual_target_coords = self.updates.get_actual_target_coords()
        points += plt.plot(actual_target_coords.long, actual_target_coords.lat,
                           'oc')

        self.plot_points.append(points)
        self.plot_num += 1

        # Remove the points older than NUM_POINTS
        if self.plot_num >= NUM_POINTS:
            for point in self.plot_points[self.plot_num - NUM_POINTS]:
                point.remove()

        # Call pause to render the changes.
        plt.pause(0.0000001)

    def pre_run_init(self):
        """
        
        """
    

        drone_positions = [self.get_tx_GPS()] + self.updates.get_rx_positions()
        self.tx_swarm = Swarming(drone_positions)

        self.pos_filter = multilateration.PosFilter(
            drones_i = drone_positions,
            z_std_drone = self.flight_parameters["COMMON"]["Z_STD_DRONE"],
            z_std_range = self.flight_parameters["COMMON"]["Z_STD_RANGE"],
            initial_target = self.target_position_GPS,
            initial_target_uncertanty = 1)

    def run(self):
        """ 
        Tx main loop.
        Receives updates from the Rxs until a new group of readings is ready
        (or until timeout occurs).
        Then performs multilateration and swarming checks to determine the
        desired target location, and sends this to each Rx.
        """
        success = True

        # Check the formation - update swarming state
        self.swarming_checks(self.tx_swarm)

        self.est_target_pos = self.perform_multilateration(self.target_position_GPS)

        self.act_target_pos = self.updates.get_actual_target_coords()
        
        self.multilateration_error = self.est_target_pos.distance3d(self.act_target_pos)
        # This line adds the multilateration into the swarm. Set it to act_target_pos to exclude it
        self.target_position_GPS = self.est_target_pos 

        desired_centre_position = self.swarming_destination(self.tx_swarm)

        # Plot the current Tx, Rx and actual target positions if needed.
        if self.should_plot:
            self.plot_positions()

        # Update the Tx's own position.
        self.tx_desired_coords = update_loc(
            desired_centre_position, TX_ID, self.get_tx_GPS())

        # Send the Tx position setpoint
        self.mavros_controller.reach_position(
            self.tx_desired_coords.lat,
            self.tx_desired_coords.long)

        return success


    def wait_for_rxs(self):
        """ Wait for a ready message to be received from each Rx, then send a
        reply once all Rxs are ready.
        """
        # Keep track of the Rx IDs which are ready.
        # Each ID is mapped to its address, so that the Tx can send a response to
        # the same addresses once all Rxs are ready.

        ready_ids = {}

        rospy.loginfo("TX: Waiting for ready messages from RXs...")
        while len(ready_ids) < NUM_RXS:
            address, empty, message = self.startup.recv_multipart()
            # The ready message is a single byte containing the Rx ID.
            rx_id = message[0]
            ready_ids[rx_id - 1] = address
            rospy.loginfo("TX: RX {} ready.".format(rx_id))

        rospy.loginfo("TX: All RXs ready.")

        input("\nAll drones ready. Start tracking?\n")

        # Let the Rxs know the Tx is ready (with an empty message).
        for address in ready_ids.values():
            self.startup.send_multipart([address, b'', b'h'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-p',
        '--plot',
        action='store_true',
        help='plots the drone and target positions on a graph during execution'
    )
    parser.add_argument(
        '-s',
        '--sim',
        action='store_true',
        help='the program will communicate with a MAVROS flight controller \
        (either simulated in Gazebo, or a real controller in a practical test)'
    )
    parser.add_argument(
        '-a',
        '--auto_arm',
        action='store_true',
        help='automatically sets mode to OFFBOARD and arms the UAV \
        (for simulation testing only, don\'t use in a practical flight!)')
    parser.add_argument(
        '-f',
        '--field',
        action='store_true',
        help='used when running tx.py on a real UAV for a field test')
    args = parser.parse_args()
    

    # If this is a field test, we should communicating with a MAVROS flight
    # controller (args.sim is True), and we shouldn't auto_arm the UAV.
    if args.field:
        assert (args.sim and not args.auto_arm)

    context = zmq.Context()
    start = time.time()

    # When running a simulation or practical flight test, perform UAV setup
    # before everything else so that the UAV is ready to fly.
    if args.sim:
        mavros_controller = MultiMavrosOffboardPosctl()
        if args.field:
            mavros_controller.set_up(-1)
            mavros_controller.setup_practical()
        else:
            mavros_controller.set_up(TX_ID)
            mavros_controller.take_off(args.auto_arm)

    tx = TransmitterUAV(context, args.plot, mavros_controller if args.sim else None)
    threshold_m = 1

    # ==============================================================================
    # Main Loop & TX State Machine
    # ==============================================================================

    loop_num = 0
    while True:
        try:
            # If the simulation is running, check for a rospy exit.
            if mavros_controller.is_shutdown():
                break

            elif tx.state == ON_STATE:
                # As soon as TX turns on set up the sockets for communicating with RXs and controller
                if tx.connect():
                    tx.state = CONECTED_STATE

            elif tx.state == CONECTED_STATE:
                if tx.target_position_GPS != None:
                    p_str = "TX is out of position. Move it {:.1f}m North and {:.1f}m East. Press 'y' to continue. Press anything else for another update\n"
                    current_pos = mavros_controller.get_current_position()
                    desired_pos = tx.tx_desired_coords
                    response = input(p_str.format(
                        current_pos.y_distance(desired_pos),
                        current_pos.x_distance(desired_pos),
                    ))
                    if response == 'y':
                        rospy.loginfo("TX: Waiting for offboard...")
                        tx.state = OFFBOARD_WAIT_STATE
                # else:
                #     rospy.logfatal("TX: Could not connect to target position server")
            

            elif tx.state == OFFBOARD_WAIT_STATE:
                if mavros_controller.wait_for_offboard():
                    input("TX: Offboard activated. Move to formation start position?\n")
                    start = time.time() # Record the time to compare with later so we don't flood the terminal with prints
                    tx.state = OFFBOARD_STATE

            elif tx.state == OFFBOARD_STATE:
                current_pos = mavros_controller.get_current_position()
                desired_pos = tx.tx_desired_coords

                if current_pos.distance(desired_pos) <= threshold_m:
                    rospy.loginfo("TX: in position. Waiting for RXs")
                    # Tell the Tx that this Rx is ready, and wait for it to respond.
                    # tx.wait_for_rxs()
                    tx.state = READY_STATE
                elif abs(time.time() - start) > 1:
                    start = time.time()
                    mavros_controller.reach_position(
                        desired_pos.lat,
                        desired_pos.long,
                        desired_pos.alt)

            elif tx.state == READY_STATE:
                if tx.receive_updates():
                    print("+===============================================")
                    states_list = tx.updates.get_states()
                    rospy.logwarn(str(states_list))
                    if all(state == READY_STATE for state in states_list):
                        if input("\nAll drones ready. Start tracking?\n") == 'y':
                            tx.state = TRACKING_STATE
                            tx.pre_run_init()

            elif tx.state == TRACKING_STATE:
                success = tx.run()
                if not success:
                    rospy.logfatal("TX: someting went wrong in run")
                    break


            if tx.state >= CONECTED_STATE:
                # Update the Rx drones if we recieve an update from all them containing the target's location
                if tx.receive_updates():
                  
                    # Send the desired centre position and the Tx position to the Rxs.
                    states_list = [tx.state] + tx.updates.get_states()

                    # Before tracking starts there will be not estimated target pos, so just send the real one
                    if tx.est_target_pos is None:
                        target_pos = tx.target_position_GPS
                    else:
                        target_pos = tx.est_target_pos

                    update = TxUpdate(states_list, target_pos, tx.get_tx_GPS())
                    tx.sender.send(update.to_bytes())

                    log_items = [
                        str(states_list),
                        str(None if tx.pos_filter is None else tx.pos_filter.get_swarm_item_pos(5)[-1, :]),  # cart_target_filt
                        str(None if tx.est_target_pos is None else tx.est_target_pos),                           # gps_target_filt
                        str(None if tx.act_target_pos is None else tx.act_target_pos),                           # gps_target_real
                        str(None if tx.pos_filter is None else tx.pos_filter.get_swarm_item_pos(0)[-1, :]),  # cart_tx_filt
                        str(tx.tx_desired_coords),                        # gps_tx_filt
                        str(tx.get_tx_GPS()),                             # gps_tx_real
                        str(None if tx.multilateration_error is None else tx.multilateration_error),                    # multilateration_error
                        str(tx.mavros_controller.state.mode)              # mode
                    ]
                    tx.logger.log_items(log_items)
                
            loop_num += 1

        except KeyboardInterrupt:
            break
    
    rospy.logfatal("we shouldnt be here ({}), ln:{} ...".format(tx.state, loop_num))
    tx.close_sockets()
    context.term()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("TX: Interrupted.")
    except Exception as e:
        rospy.logfatal(traceback.format_exc())
    finally:
        sys.exit(0)

