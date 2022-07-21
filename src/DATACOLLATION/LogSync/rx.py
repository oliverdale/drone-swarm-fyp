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

from common import *
from gps import GPSCoord
from packets import RxUpdate, TxUpdate
import swarming_logic
from mavros_offboard_posctl import MultiMavrosOffboardPosctl
import datacollation2V3wTiming

# Timeout period for receiving updates from the Tx.
TIMEOUT_S = 3

# Extra time to wait for the first Tx update, which could take longer due to
# startup processes.
INITIAL_TIMEOUT_S = 2

# Timeout period for receiving target coords from the server.
TARGET_POS_TIMEOUT_S = 3


class TimeoutException(Exception):
    """ Raised when a timeout occurs, waiting for either updates from the Tx
    or target positions from the target position server.
    """
    pass


class ReceiverUAV:
    def __init__(self, context, rx_id, mavros_controller=None):
        # ID of this Rx, in the range 1 to NUM_RXS.
        self.rx_id = rx_id

        # Create a PUSH socket to send updates to the Tx.
        self.sender = context.socket(zmq.PUSH)
        self.sender.setsockopt(zmq.LINGER, 0)  # Exit despite unsent updates.
        self.sender.connect("tcp://{}:{}".format(TX_IP_ADDRESS,
                                                 TX_RECEIVE_PORT))

        # Create a SUB socket to receive updates from the Tx.
        self.receiver = context.socket(zmq.SUB)
        self.receiver.connect("tcp://{}:{}".format(TX_IP_ADDRESS,
                                                   TX_SEND_PORT))
        self.receiver.setsockopt_string(zmq.SUBSCRIBE, "")

        # Crete a SUB socket for receiving the target coordinates.
        self.target_pos_socket = context.socket(zmq.SUB)
        self.target_pos_socket.connect("tcp://{}:{}".format(
            TARGET_POS_IP_ADDRESS, TARGET_POS_PORT))
        self.target_pos_socket.setsockopt(zmq.SUBSCRIBE, b"")

        # Check if we are communicating with a MAVROS flight controller (either
        # simulated or real).
        self.mavros_controller = mavros_controller
        self.sim_running = self.mavros_controller is not None

        # TODO: hacked this together for the practical test - fix it!!!
        # If the sim is running, don't use the hard-coded Rx and Tx start positions.
        # We should already be in formation now, so we can just use the current
        # Rx and Tx positions.

        # The Rx UAV needs to know the Tx's UAV to emulate the range readings.
        # TODO: this is hack which just gets the current target posision from
        # the server and assumes this is also the Tx's position. This should
        # be done in a nicer way.
        print("Receiving start position from target position server...")
        self.tx_coords = GPSCoord(
            *struct.unpack("dd", self.target_pos_socket.recv()))

        # Initialise the variable storing the Rx UAV's current posision.
        if self.sim_running:
            # If we are using a MAVROS controller, the UAV should already be at
            # it's start position in the formation, so we can just get its
            # current position.
            self.rx_desired_coords = self.mavros_controller.get_current_position(
            )
        else:
            # If there is no MAVROS controller, rx_desired_coords represents the
            # current "ideal" position of the Rx, based on the desired location
            # output by the swaming logic. If no simulation is running, this is
            # used as the current position of the Rx (see get_rx_coords()).

            # TODO: find a better way to initialise this than calculating it
            # from the Tx posiion.
            self.rx_desired_coords = swarming_logic.update_loc(
                self.tx_coords, self.rx_id, None)
            assert (self.rx_desired_coords is not None)

        # A sequence number for the readings taken. Sent to the Tx, which uses
        # it to group readings from all the Rxs which have the same number.
        self.reading_number = 0

        # Poller to simultaneously wait for Tx updates and target coordinates.
        self.poller = zmq.Poller()
        self.poller.register(self.receiver, zmq.POLLIN)
        self.poller.register(self.target_pos_socket, zmq.POLLIN)

    def get_rx_coords(self):
        """ Returns the current position of the Rx drone. If a simulation is
        running, get the position from there. Otherwise, return the "ideal"
        current position, based on multilateration and swarming logic.
        """
        if self.sim_running:
            return self.mavros_controller.get_current_position()
        else:
            return self.rx_desired_coords

    def calculate_range(self, target_coords):
        """ Calculates the range reading in meters as the distance from the Rx to
        the target, plus the distance from the target to the Tx.
        """
        return (self.get_rx_coords().distance(target_coords) +
                target_coords.distance(self.tx_coords))

    def receive_update(self):
        """ Receive an update from the Tx, and return the new desired location of
        this Rx, and the current position of the Tx.
        """
        # Called when socket is ready for reading, so non-blocking recv can be used.
        update = TxUpdate.from_bytes(self.receiver.recv(flags=zmq.NOBLOCK))

        # Calculate the desired location for this Rx based on the swarming logic.
        self.rx_desired_coords = swarming_logic.update_loc(
            update.target_coords, self.rx_id, self.get_rx_coords())
        self.tx_coords = update.tx_coords

    def send_update(self, target_coords):
        """ Send an update to the Tx containing the Rx position and range. """

        # print("Expected target coords {}".format(target_coords))

        # Calculate the emulated range based on the Rx, Tx and target positions.
        range_reading = self.calculate_range(target_coords)

        update = RxUpdate(self.rx_id, time.time(), self.reading_number,
                          self.get_rx_coords(), range_reading, target_coords)
        self.reading_number += 1

        self.sender.send(update.to_bytes())

    def run(self):
        """ The Rx main loop. Receives updates from the Tx whenever they arrive,
        and sends updates to the Tx whenever new target coords are received from
        the GPS server.
        """
        # Last time an update from the Tx was received, to check for timeout.
        last_update_received_time = time.time() + INITIAL_TIMEOUT_S

        # Last time new target coords were received from the server.
        last_target_pos_time = time.time()
        datacollation2V3wTiming.log_init_des_start(self.rx_id)
        datacollation2V3wTiming.log_init_cur_start(self.rx_id)
        logger = datacollation2V3wTiming.LogGPS(self.rx_desired_coords.lat, self.rx_desired_coords.long, self.rx_id, time.time()) #, cur_time_rx)
        logger_real = datacollation2V3wTiming.LogGPS(self.rx_desired_coords.lat, self.rx_desired_coords.long, self.rx_id, time.time()) #, cur_time_rx)
        #time_logger_des = datacollation2V3.LogTimes(0.0, 0.0, 0.0, 0.0)
        while True:
            # If the simulation is running, check for a rospy exit.
            if self.sim_running and self.mavros_controller.is_shutdown():
                break

            sockets = dict(
                self.poller.poll(timeout=min(TIMEOUT_S, TARGET_POS_TIMEOUT_S)))

            # Send an update if we have new target coords from the GPS server.
            if self.target_pos_socket in sockets:
                last_target_pos_time = time.time()
                message = self.target_pos_socket.recv(zmq.NOBLOCK)
                lat, lon = struct.unpack("dd", message)
                self.send_update(GPSCoord(lat, lon))
            else:
                # No new target coords, check if timeout occurred.
                if time.time() >= last_target_pos_time + TARGET_POS_TIMEOUT_S:
                    raise TimeoutException(
                        "ERROR: no target coords recevied in {} s.".format(
                            TARGET_POS_TIMEOUT_S))

            # Receive an update from the Tx if one is ready.
            t_rec_sig = time.time()
            if self.receiver in sockets:
                last_update_received_time = time.time()
                # Update desired Rx position and Tx position based on update.
                self.receive_update()

                # Change the Rx position setpoint in the simulation if running.
                if self.sim_running:
                    self.mavros_controller.reach_position(
                        self.rx_desired_coords.lat,
                        self.rx_desired_coords.long)
            else:
                # Didn't receive an update from the Tx, check if timeout occurred.
                if time.time() >= last_update_received_time + TIMEOUT_S:
                    raise TimeoutException(
                        "Timeout occurred. No updates from Tx in {} s.".format(
                            TIMEOUT_S))
            #print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            datacollation2V3wTiming.LogGPS.time_check_des(logger, self.rx_id, self.rx_desired_coords.lat, self.rx_desired_coords.long, t_rec_sig)
            rx_cords_cur = self.get_rx_coords()
            t_rec_sig_cur_rx = time.time()
            datacollation2V3wTiming.LogGPS.time_check_cur(logger_real, self.rx_id, rx_cords_cur, 0, t_rec_sig_cur_rx)


def wait_for_tx(context, rx_id):
    """ Send the Tx a ready message containg the Rx ID and wait for a response. """
    startup = context.socket(zmq.REQ)
    startup.setsockopt(zmq.LINGER, 0)  # exit if startup process interrupted
    startup.connect("tcp://{}:{}".format(TX_IP_ADDRESS, TX_STARTUP_PORT))
    startup.send(bytes([rx_id]))

    print("Waiting for ready message from Tx...")
    startup.recv()
    startup.close()
    print("Tx ready.")


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

    # If this is a field test, we should communicating with a MAVROS flight
    # controller (args.sim is True), and we shouldn't auto_arm the UAV.
    if args.field:
        assert (args.sim and not args.auto_arm)

    context = zmq.Context()

    # When running a simulation or practical flight test, perform UAV setup
    # before everything else so that the UAV is ready to fly.
    if args.sim:
        mavros_controller = MultiMavrosOffboardPosctl()
        if args.field:
            mavros_controller.set_up(-1)
            mavros_controller.setup_practical()
        else:
            mavros_controller.set_up(args.rx_id)
            mavros_controller.take_off(args.auto_arm)
        move_into_formation(context, mavros_controller, args.rx_id)

    # Tell the Tx that this Rx is ready, and wait for it to respond.
    wait_for_tx(context, args.rx_id)

    rx = ReceiverUAV(context, args.rx_id,
                     mavros_controller if args.sim else None)

    try:
        rx.run()
    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception as e:
        print(e)
    finally:
        print("Exiting...")
        sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # This handles a keyboard interrupt during the startup process.
        print("Interrupted. Exiting...")
        sys.exit(0)
