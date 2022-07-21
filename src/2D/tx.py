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
import rospy

from common import *
import multilateration
import swarming_logic
from gps import GPSCoord
from packets import RxUpdate, TxUpdate
from mavros_offboard_posctl import MultiMavrosOffboardPosctl
from swarming_logic import Swarming
from update_store import UpdateStore
import datacollation2V3wTiming

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
        # PULL socket for receiving updates from the Rxs.
        self.receiver = context.socket(zmq.PULL)
        self.receiver.bind("tcp://*:{}".format(TX_RECEIVE_PORT))

        # PUB socket for sending target position estimates to the Rxs.
        self.sender = context.socket(zmq.PUB)
        self.sender.bind("tcp://*:{}".format(TX_SEND_PORT))

        # Check if a MAVROS flight controller is being used.
        self.mavros_controller = mavros_controller
        self.sim_running = self.mavros_controller is not None

        # Initialise the variable storing the Tx UAV's current position.
        if self.sim_running:
            # If we are using a MAVROS controller, the UAV should already be at
            # it's start position in the formation, so we can just get its
            # current position.
            self.tx_desired_coords = self.mavros_controller.get_current_position(
            )

        else:
            # If there is no MAVROS controller, tx_desired_coords represents the
            # current "ideal" position of the Tx, based on the desired location
            # output by the swaming logic. If no simulation is running, this is
            # used as the current position of the Rx (see get_rx_coords()).

            # TODO: this is currently initialised by receiving the target
            # position from the server and assuming this is the desired position
            # of the Tx UAV. This should be done in a nicer way.
            socket = context.socket(zmq.SUB)
            socket.connect("tcp://{}:{}".format(TARGET_POS_IP_ADDRESS,
                                                TARGET_POS_PORT))
            socket.setsockopt(zmq.SUBSCRIBE, b"")
            rospy.loginfo("Receiving start position from target position server...")
            self.tx_desired_coords = GPSCoord(
                *struct.unpack("dd", socket.recv()))
            socket.close()

        # Store all the updates received from the Rxs.
        self.updates = UpdateStore()

        # Store all the estimated target positions, so they can possibly be used
        # to recover if the target is lost.
        # TODO: only store positions temporarily then write them to a log file?
        self.target_positions = []

        # Poller required to set a timeout on receiving from the receiver socket.
        self.poller = zmq.Poller()
        self.poller.register(self.receiver, zmq.POLLIN)

        # Create a graph to plot the drone and target positions, if necessary.
        self.should_plot = should_plot
        if self.should_plot:
            plt.axis(self.get_plot_limits())
            # Keep a list of points so we can remove the old ones later. Entry
            # is a list of points, one for each UAV and the target.
            self.plot_points = []
            # How many times we've plotted points so far.
            self.plot_num = 0

    def get_plot_limits(self):
        min_point = self.tx_desired_coords.add_x_offset(
            PLOT_RANGE_M).add_y_offset(PLOT_RANGE_M)
        max_point = self.tx_desired_coords.add_x_offset(
            -PLOT_RANGE_M).add_y_offset(-PLOT_RANGE_M)

        return (min_point.long, max_point.long, min_point.lat, max_point.lat)

    def get_tx_coords(self):
        """ Returns the current position of the Tx drone. If a simulation is
        running, get the position from there. Otherwise, return the "ideal"
        current position, based on multilateration and swarming logic.
        """
        if self.sim_running:
            return self.mavros_controller.get_current_position()
        else:
            return self.tx_desired_coords

    def receive_updates(self, timeout_time):
        """ Repeatedly receive and store updates from the Rxs, returning once
        a new full group of readings is ready. Raises a TimeoutException if
        this doesn't happen before the timeout_time
        """
        while True:
            # Time interval until timeout_time in ms.
            timeout_interval = max(0, 1000 * (timeout_time - time.time()))
            sockets = dict(self.poller.poll(timeout=timeout_interval))
            if self.receiver in sockets:
                message = self.receiver.recv(flags=zmq.NOBLOCK)
                update = RxUpdate.from_bytes(message)
                if self.updates.store(update):
                    return
            else:
                # Print error message including the the time since an update
                # was received from each Rx, to help diagnose the timeout.
                message = "ERROR: Timeout occurred. Times since last update:\n"
                times_since_update = [
                    time.time() - last_time
                    for last_time in self.updates.get_last_update_times()
                ]
                for rx_id in range(1, NUM_RXS + 1):
                    message += "Rx {}: {:.3f} s\n".format(
                        rx_id, times_since_update[rx_id - 1])
                raise TimeoutException(message)

    def perform_multilateration(self):
        """ Returns the target position estimated by the multilateration module.
        """
        rx_positions = self.updates.get_rx_positions()
        ranges = self.updates.get_ranges()
        result = multilateration.estimate_target_position(
            self.get_tx_coords(), rx_positions, ranges, NUM_RXS)
        return result

    def swarming_checks(self, tx_swarm):
        """ Returns the desired centre position of the formation. 
        If formation is fine, output the target position, 
        otherwise outputs the averaged centre of the formation. 
        Discards unreasonable target GPS values. """
        drone_positions = [self.get_tx_coords()
                           ] + self.updates.get_rx_positions()

        tx_swarm.update_drones(drone_positions)
        tx_swarm.swarming_checks()

    def swarming_destination(self, tx_swarm):
        target_coords = self.target_positions[-1]
        tx_swarm.update_target(target_coords)
        tx_swarm.update_swarming_destination()
        return tx_swarm.dest_coord

    def plot_positions(self):
        """ Plot the current positions of the Tx and the Rxs, and the actual
        target position. """
        colors = ['k', 'b', 'g', 'r', 'm']
        positions = [self.get_tx_coords()] + self.updates.get_rx_positions()

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

    def run(self):
        """ Tx main loop.
        Receives updates from the Rxs until a new group of readings is ready
        (or until timeout occurs).
        Then performs multilateration and swarming checks to determine the
        desired target location, and sends this to each Rx.
        """
        # Wait until at least one update is received from each Rx.
        self.receive_updates(time.time() + TIMEOUT_S)
        last_updates_received_time = time.time()

        drone_positions = [self.get_tx_coords()
                           ] + self.updates.get_rx_positions()
        tx_swarm = Swarming(drone_positions)
        datacollation2V3wTiming.log_init_des_start(TX_ID)
        datacollation2V3wTiming.log_init_cur_start(TX_ID)
        transmitter = datacollation2V3wTiming.LogGPS(self.tx_desired_coords.lat, self.tx_desired_coords.long, TX_ID, time.time()) #, cur_time_tx)
        transmitter_real = datacollation2V3wTiming.LogGPS(self.tx_desired_coords.lat, self.tx_desired_coords.long, TX_ID, time.time()) #, cur_time_tx)
        while True:
            # If the simulation is running, check for a rospy exit.
            if self.sim_running and self.mavros_controller.is_shutdown():
                break

            loop_start_time = time.time()

            self.receive_updates(last_updates_received_time + TIMEOUT_S)
            last_updates_received_time = time.time()

            # Check the formation - update swarming state
            self.swarming_checks(tx_swarm)

            target_coords = self.perform_multilateration()
            self.target_positions.append(target_coords)
            rospy.loginfo("Multilateration error {:.2}".format(
                target_coords.distance(
                    self.updates.get_actual_target_coords())))

            desired_centre_position = self.swarming_destination(tx_swarm)

            # Plot the current Tx, Rx and actual target positions if needed.
            if self.should_plot:
                self.plot_positions()

            # Send the desired centre position and the Tx position to the Rxs.
            update = TxUpdate(desired_centre_position, self.get_tx_coords())
            # new
            t_rec_sig = time.time()
            self.sender.send(update.to_bytes())

            # Update the Tx's own position.
            self.tx_desired_coords = swarming_logic.update_loc(
                desired_centre_position, TX_ID, self.get_tx_coords())

            # Send the Tx position setpoint for the simulation if it's running.
            if self.sim_running:
                self.mavros_controller.reach_position(
                    self.tx_desired_coords.lat, self.tx_desired_coords.long)

            #time_sample, lat_str_gps_true, lon_str_gps_true = line.strip().split(",")
            #gps_tx_coords = GPSCoord(self.get_tx_coords())
            #gps_tx_coords = GPSCoord.nmea_to_dd()
            #print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            datacollation2V3wTiming.LogGPS.time_check_des(transmitter, TX_ID, self.tx_desired_coords.lat, self.tx_desired_coords.long, t_rec_sig)
            coords_cur = self.get_tx_coords()
            t_rec_sig_cur = time.time()
            datacollation2V3wTiming.LogGPS.time_check_cur(transmitter_real, TX_ID, coords_cur, 0, t_rec_sig_cur)  #, gps_tx_coords.long)
            print()


def wait_for_rxs(context):
    """ Wait for a ready message to be received from each Rx, then send a
    reply once all Rxs are ready.
    """
    startup = context.socket(zmq.ROUTER)
    startup.bind("tcp://*:{}".format(TX_STARTUP_PORT))

    # Keep track of the Rx IDs which are ready.
    # Each ID is mapped to its address, so that the Tx can send a response to
    # the same addresses once all Rxs are ready.
    ready_ids = {}

    rospy.loginfo("Waiting for ready messages from Rxs...")
    while len(ready_ids) < NUM_RXS:
        address, empty, message = startup.recv_multipart()
        # The ready message is a single byte containing the Rx ID.
        rx_id = message[0]
        ready_ids[rx_id - 1] = address
        rospy.loginfo("Rx {} ready.".format(rx_id))

    rospy.loginfo("All Rxs ready.")

    # Let the Rxs know the Tx is ready (with an empty message).
    for address in ready_ids.values():
        startup.send_multipart([address, b'', b''])

    startup.close()


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
    args = parser.parse_args()

    context = zmq.Context()

    # When running a simulation or practical flight test, perform UAV setup
    # before everything else so that the drone is ready to fly.
    if args.sim:
        mavros_controller = MultiMavrosOffboardPosctl()
        mavros_controller.set_up(TX_ID)
        mavros_controller.take_off(args.auto_arm)
        move_into_formation(context, mavros_controller, TX_ID)

    # Wait until all Rxs have sent a ready message.
    wait_for_rxs(context)

    tx = TransmitterUAV(context, args.plot,
                        mavros_controller if args.sim else None)

    try:
        tx.run()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted.")
    except Exception as e:
        print(e)
    finally:
        rospy.loginfo("Exiting...")
        sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted. Exiting...")
        sys.exit(0)
