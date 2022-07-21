"""
filename: rx.py
author: Reka Norman
date: 16th October 2020
description: The main program which runs on the Rx UAVs.
"""

import zmq
import time
import sys
import argparse
import struct
import traceback

from zmq.sugar.context import T

import logger
from common import *
import rospy
from gps import GPSCoord
from packets import RxUpdate, TxUpdate
from mavros_offboard_posctl import MultiMavrosOffboardPosctl


# Timeout period for receiving updates from the Tx.
TIMEOUT_S = 3

# Extra time to wait for the first Tx update, which could take longer due to
# startup processes.
INITIAL_TIMEOUT_S = 2


TARGET_POS_TIMEOUT_S = 3


class TimeoutException(Exception):
    """ Raised when a timeout occurs, waiting for either updates from the Tx
    or target positions from the target position server.
    """
    pass


class ReceiverUAV:
    def __init__(self, context, rx_id, mavros_controller=None):
        # ID of this Rx, in the range 1 to NUM_RXS.
        self.context = context
        self.rx_id = rx_id
        self.mavros_controller = mavros_controller
        self.state = ON_STATE

        self.tx_GPS = None
        self.target_GPS = None
        self.rx_desired_coords = None

        flight_parameters = get_parameters()
        items_to_log = ['gps_pos_filt','gps_pos_curr', 'state', 'mode']
        self.logger = logger.Logger(rx_id, items_to_log)

        self.tx_ip = flight_parameters['0']['IP_ADDRESS']
        self.target_pos_timeout = flight_parameters['COMMON']['TARGET_POS_TIMEOUT']

        self.tx_startup_port = flight_parameters['COMMON']['STARTUP_PORT']
        self.tx_recieve_port = flight_parameters['COMMON']['RECEIVE_PORT']
        self.tx_send_port = flight_parameters['COMMON']['SEND_PORT']

        self.target_ip = flight_parameters['COMMON']['TARGET_POS_IP_ADDRESS']
        self.target_port = flight_parameters['COMMON']['TARGET_POS_PORT']

        # A sequence number for the readings taken. Sent to the Tx, which uses
        # it to group readings from all the Rxs which have the same number.
        self.reading_number = 0

        self.receiver = None
        self.sender = None
        self.target_pos_socket = None
        self.controller = None
        self.startup = None

        self.tx_state = None

    def close_sockets(self):
        """
        Make sure all the sockets are closed when RX terminates
        """
        if self.receiver is not None:
            self.receiver.close()
        if self.sender is not None:
            self.sender.close()
        if self.target_pos_socket is not None:
            self.target_pos_socket.close()
        if self.controller is not None:
            self.controller.close()
        if self.startup is not None:
            self.startup.close()
        rospy.logwarn("RX{}: Closed sockets".format(self.rx_id))


    def connect(self):
        """
        
        """
        success = True
        try:
            # Create a PUSH socket to send updates to the Tx.
            self.sender = self.context.socket(zmq.PUSH)
            self.sender.setsockopt(zmq.LINGER, 0)  # Exit despite unsent updates.
            self.sender.connect("tcp://{}:{}".format(self.tx_ip, self.tx_recieve_port))
        except Exception as e:
            rospy.logfatal("RX{}: Could not setup sender connection".format(self.rx_id))
            rospy.logfatal(traceback.format_exc())
            success = False

        try:
            # Create a SUB socket to receive updates from the Tx.
            self.receiver = self.context.socket(zmq.SUB)
            self.receiver.connect("tcp://{}:{}".format(self.tx_ip, self.tx_send_port))
            self.receiver.setsockopt(zmq.SUBSCRIBE, b"")
        except Exception as e:
            rospy.logfatal("RX{}: Could not setup reciever connection".format(self.rx_id))
            rospy.logfatal(traceback.format_exc())
            success = False

        try:
            # Crete a SUB socket for receiving the target coordinates.
            self.target_pos_socket = self.context.socket(zmq.SUB)
            self.target_pos_socket.connect("tcp://{}:{}".format(self.target_ip , self.target_port))
            self.target_pos_socket.setsockopt(zmq.SUBSCRIBE, b"")
        except Exception as e:
            rospy.logfatal("RX{}: Could not setup target pos connection".format(self.rx_id))
            rospy.logfatal(traceback.format_exc())
            success = False

        try:
            self.startup = self.context.socket(zmq.REQ)
            self.startup.setsockopt(zmq.LINGER, 0)  # exit if startup process interrupted
            self.startup.connect("tcp://{}:{}".format(self.tx_ip, self.tx_startup_port))
        except Exception as e:
            rospy.logfatal("RX{}: Could not setup startup connection".format(self.rx_id))
            rospy.logfatal(traceback.format_exc())
            success = False
        
        if success:
            # Poller to simultaneously wait for Tx updates and target coordinates.
            self.poller = zmq.Poller()
            self.poller.register(self.receiver, zmq.POLLIN)
            self.poller.register(self.target_pos_socket, zmq.POLLIN)
        else:
            rospy.logfatal("RX{}: Could not setup sockets".format(self.rx_id))
            self.close_sockets()

        return success


    def get_first_pos(self):
        """
        
        """
        success = True
        
        rospy.loginfo("RX{}: Receiving start position from target position server".format(self.rx_id))
        if self.target_GPS is None:
            self.target_GPS = GPSCoord(*struct.unpack("ddd", self.target_pos_socket.recv()))
            # Initialise the variable storing the Rx UAV's current posision relative to the initial target position
            self.rx_desired_coords = update_loc(self.target_GPS, self.rx_id)
            # The Rx UAV needs to know the Tx's UAV location to emulate the range readings. Get the first range reading by assuming the TX is above the target.
            self.tx_GPS = update_loc(self.target_GPS, TX_ID)

        assert (self.rx_desired_coords is not None)

        return success


    def get_rx_coords(self):
        """ 
        Returns the current position of the Rx drone.
        """
        return self.mavros_controller.get_current_position()


    def calculate_range(self, target_coords):
        """ 
        Calculates the range reading in meters as the distance from the Rx to
        the target, plus the distance from the target to the Tx.
        """
        return (self.get_rx_coords().distance3d(target_coords) + target_coords.distance3d(self.tx_GPS))




    def send_update(self):
        """ Send an update to the Tx containing the Rx position and range. """
        # Calculate the emulated range based on the Rx, Tx and target positions.
        if self.target_GPS is not None:
            range_reading = self.calculate_range(self.target_GPS)
            update = RxUpdate(
                rx_id = self.rx_id, 
                state = self.state, 
                timestamp = time.time(),
                seq_no = self.reading_number,
                rx_coords = self.get_rx_coords(),
                range_reading = range_reading,
                target_coords = self.target_GPS)
        else:
            update = RxUpdate(
                rx_id = self.rx_id, 
                state = self.state, 
                timestamp = time.time(),
                seq_no = self.reading_number,
                rx_coords = self.get_rx_coords(),
                range_reading = -1,
                target_coords = GPSCoord(-1,-1,-1))

        self.reading_number += 1
        self.sender.send(update.to_bytes())

    def receive_tx_update(self):
        """ Receive an update from the Tx, and return the new desired location of
        this Rx, and the current position of the Tx.
        """
        sockets = {}
        try:
            sockets = dict(self.poller.poll(timeout=100))
        except TimeoutException:
            rospy.logfatal("RX{}: Timeout occurred. No updates from Tx in {} s.".format(self.rx_id, TIMEOUT_S))
            self.close_sockets()
        except Exception as e:
            rospy.logfatal(traceback.format_exc())
            self.close_sockets()

        if self.receiver in sockets:
            # Called when socket is ready for reading, so non-blocking recv can be used.
            update = TxUpdate.from_bytes(self.receiver.recv(flags=zmq.NOBLOCK))
            # Calculate the desired location for this Rx based on the swarming logic.
            self.rx_desired_coords = update_loc(update.target_coords, self.rx_id, self.get_rx_coords())
            self.tx_state = update.states[0]
            self.tx_GPS = update.tx_GPS
               

    def recieve_target_GPS(self):
        
        target_recieved = False
        sockets = {}
        try:
            sockets = dict(self.poller.poll(timeout=100))
        except TimeoutException:
            rospy.logwarn("RX{}: ERROR no target coords recevied in {} s.".format(self.rx_id, TIMEOUT_S))
            self.close_sockets()
        except Exception as e:
            rospy.logfatal(traceback.format_exc())
            self.close_sockets()

        if self.target_pos_socket in sockets:
            target_recieved = True
            self.target_GPS = GPSCoord(*struct.unpack("ddd", self.target_pos_socket.recv()))
            # Initialise the variable storing the Rx UAV's current posision relative to the initial target position
            if self.rx_desired_coords is None:
                self.rx_desired_coords = update_loc(self.target_GPS, self.rx_id)
            if self.tx_GPS is None:
                # The Rx UAV needs to know the Tx's UAV location to emulate the range readings. Get the first range reading by assuming the TX is above the target.
                self.tx_GPS = update_loc(self.target_GPS, TX_ID)

            

        return target_recieved

                
    def run(self):
        """ The Rx main loop. Receives updates from the Tx whenever they arrive,
        and sends updates to the Tx whenever new target coords are received from
        the GPS server.
        """
        success = True

     

        return success


    def wait_for_tx(self):
        """ Send the Tx a ready message containg the Rx ID and wait for a response. """
        
        self.startup.send(bytes([self.rx_id]))

        rospy.loginfo("RX{}: Waiting for ready message from Tx...".format(self.rx_id))
        try:
            #check for a message
            message = self.startup.recv()
            #a message has been received
            rospy.loginfo("RX{}: Tx ready: {}".format(self.rx_id, message))
        except Exception as e:
            rospy.logfatal("RX{}: {}".format(self.rx_id, traceback.format_exc()))



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'rx_id',
        type=int,
        choices=range(1, NUM_RXS + 1),
        help='the ID of this UAV (Rx UAVs should be numbered starting from 1)')
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
        help='used when running rx.py on a real UAV for a field test')
    args = parser.parse_args()

    # If this is a field test we shouldn't auto_arm the UAV.
    if args.field:
        assert (not args.auto_arm)

    context = zmq.Context()
    start = time.time()

    # When running a simulation or practical flight test, perform UAV setup
    # before everything else so that the UAV is ready to fly.
    mavros_controller = MultiMavrosOffboardPosctl()
    if args.field:
        mavros_controller.set_up(-1)
        mavros_controller.setup_practical()
    else:
        mavros_controller.set_up(args.rx_id)
        mavros_controller.take_off(args.auto_arm)

    rx = ReceiverUAV(context, args.rx_id, mavros_controller if args.sim else None)
    threshold_m = 1

    # ==============================================================================
    # Main Loop & RX State Machine
    # ==============================================================================

    loop_num = 0
    while True:
        try:
            # If the simulation is running, check for a rospy exit.
            if mavros_controller.is_shutdown():
                break

            if rx.state == ON_STATE:
                # As soon as TX turns on set up the sockets for communicating with RXs and controller
                if rx.connect():
                    rospy.loginfo("RX{}: Waiting for target server".format(rx.rx_id))
                    rx.state = CONECTED_STATE

            elif rx.state == CONECTED_STATE:
                if rx.rx_desired_coords != None:
                    p_str = "RX{} is out of position. Move it {:.1f}m North and {:.1f}m East. Press 'y' to continue. Press anything else for another update\n"
                    current_pos = mavros_controller.get_current_position()
                    desired_pos = rx.rx_desired_coords
                    response = input(p_str.format(
                        rx.rx_id,
                        current_pos.y_distance(desired_pos),
                        current_pos.x_distance(desired_pos),
                    ))
                    if response == 'y':
                        rospy.loginfo("RX{}: Waiting for offboard...".format(rx.rx_id))
                        rx.state = OFFBOARD_WAIT_STATE
   

            elif rx.state == OFFBOARD_WAIT_STATE:
                if mavros_controller.wait_for_offboard():
                    input("RX{}: Offboard activated. Move to formation start position?\n".format(rx.rx_id))
                    start = time.time() # Record the time to compare with later so we don't flood the terminal with prints
                    rx.state = OFFBOARD_STATE

            elif rx.state == OFFBOARD_STATE:
                current_pos = mavros_controller.get_current_position()
                desired_pos = rx.rx_desired_coords

                if current_pos.distance(desired_pos) <= threshold_m:
                    rospy.loginfo("RX{}: In position. Waiting for TX".format(rx.rx_id))
                    # Tell the Tx that this Rx is ready
                    rx.state = READY_STATE
                elif abs(time.time() - start) > 1:
                    start = time.time()
                    mavros_controller.reach_position(
                        desired_pos.lat,
                        desired_pos.long,
                        desired_pos.alt)
            
            elif rx.state == READY_STATE:
                if rx.tx_state == READY_STATE:
                    rx.state = TRACKING_STATE

            elif rx.state == TRACKING_STATE:
                rx.mavros_controller.reach_position(
                    rx.rx_desired_coords.lat,
                    rx.rx_desired_coords.long)


            if rx.state >= CONECTED_STATE:
                # Update the target location if we recieve an update from the target server
                if rx.recieve_target_GPS():
                    rospy.logwarn("RX{} sending update".format(rx.rx_id))
                    rx.send_update()
                
                # Update desired Rx position and Tx position based on update.
                rx.receive_tx_update()
   
                log_items = [str(rx.rx_desired_coords),           # gps_pos_filt
                            str(rx.get_rx_coords()),              # gps_pos_curr
                            str(state_strs[rx.state]),            # state
                            str(rx.mavros_controller.state.mode)] # mode
                rx.logger.log_items(log_items)

            loop_num += 1
        
 
        except KeyboardInterrupt:
            break
    
    rospy.logfatal("RX{}: Loop closed after {} iterations".format(rx.rx_id, rx.state, loop_num))
    rx.close_sockets()
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
