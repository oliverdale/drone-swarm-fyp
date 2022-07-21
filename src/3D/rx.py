"""
filename: rx.py
author: Rowan Sinclair
date: 30th September 2021
description:
    The main file which is run on each of the RX drones. Contains the logic for sending and reciving updates 
    and the state machine which controls the drone
"""

# External Libraries
import sys
import rospy
import traceback
import time
import struct
import argparse
import zmq
# Our stuff
import logger
from failsafes import Failsafes
from common import *
from gps import GPSCoord
from packets import ContUpdate, RxUpdate, TxUpdate
from mavros_offboard_posctl import MultiMavrosOffboardPosctl


class ReceiverUAV:
    def __init__(self, context, rx_id, mavros_controller, is_local=False):

        # ID of this Rx, in the range 1 to NUM_RXS.
        self.context = context
        self.rx_id = rx_id
        self.mavros_controller = mavros_controller
        self.is_local = is_local
        
        self.state = ON_STATE
        self.reading_number = 0 # To synchronize readings between drones 

        # These are recieved from the tx drone or the target server
        self.tx_state = None
        self.tx_GPS = None
        self.target_GPS = None
        self.desired_GPS = None
        self.current_GPS = self.mavros_controller.get_current_position()

        self.tx_timeout = None
        self.target_timeout = None
        self.controller_timeout = None

        # Get the ports, IP address and other constants from the json file
        fp = get_parameters(is_local) # Flight parameters
        self.fp = fp
        self.failsafes = Failsafes(is_local)

        # Extract the connection info from the json
        self.sockets = { 'target':None, 'sender':None, 'receiver':None}  #'controller':None,
        self.ips = {
            # 'controller':fp['COMMON']['CONTROLLER_IP_ADDRESS'],
            'target':fp['COMMON']['TARGET_POS_IP_ADDRESS'],
            'sender':fp['0']['IP_ADDRESS'], 
            'receiver':fp['0']['IP_ADDRESS']}

        self.ports = {
            # 'controller':fp['COMMON']['CONTROLLER_PORT'],
            'target':fp['COMMON']['TARGET_POS_PORT'],
            'sender':fp['COMMON']['RECEIVE_PORT'], # Sending to the receive port on the TX
            'receiver':fp['COMMON']['SEND_PORT']}  # Receiveing from the send port on the TX

        self.conn_info = {
            # 'controller':(zmq.SUB,zmq.SUBSCRIBE, b""),
            'target':(zmq.SUB,zmq.SUBSCRIBE, b""),
            'sender':(zmq.PUSH, zmq.LINGER, 0), 
            'receiver':(zmq.SUB,zmq.SUBSCRIBE, b"")}
        
        # Set up the logger
        items_to_log = ['state', 'mode', 'reading_number', 'desired_GPS', 'current_GPS', 'target_GPS']
        self.logger = logger.Logger(rx_id, items_to_log)


    def set_up_conections(self):
        """
        Sets up the conections to the target position socket, the tx drone sender socket,
        the tx drone reciever socket, and the controller. Returns True if successful
        """
        success = True
        for key in self.sockets.keys():
            try:
                # Create a PUSH socket to send updates to the Tx.
                self.sockets[key] = self.context.socket(self.conn_info[key][0])
                self.sockets[key].setsockopt(self.conn_info[key][1], self.conn_info[key][2])
                self.sockets[key].connect("tcp://{}:{}".format(self.ips[key], self.ports[key]))
            except Exception as e:
                rospy.logfatal("RX{}: Could not setup {} connection".format(self.rx_id, key))
                rospy.logfatal(traceback.format_exc())
                success = False

        if success:
            # Poller to simultaneously wait for Tx updates and target coordinates.
            self.poller = zmq.Poller()
            self.poller.register(self.sockets['receiver'], zmq.POLLIN)
            self.poller.register(self.sockets['target'], zmq.POLLIN)
            # self.poller.register(self.sockets['controller'], zmq.POLLIN)
        else:
            rospy.logfatal("RX{}: Could not setup sockets".format(self.rx_id))
            success = False

        return success

    def close_sockets(self):
        """
        Make sure all the sockets are closed when RX terminates
        """
        for socket in self.sockets.values():
            if socket is not None:
                socket.close()

        rospy.logwarn("RX{}: Closed sockets".format(self.rx_id))

    def get_rx_GPS(self):
        """ 
        Returns the current position of the Rx drone and sets the attribute
        """
        self.current_GPS = self.mavros_controller.get_current_position()
        return self.current_GPS

    def calculate_range(self):
        """ 
        Calculates the range reading in meters as the distance from the Rx to
        the target, plus the distance from the target to the Tx.
        """
        return (self.get_rx_GPS().distance3d(self.target_GPS) + self.target_GPS.distance3d(self.tx_GPS))
    
    def receive_updates(self):
        """
        Checks for updates on all the sockets and updates the associated attributes.
        Returns a list of 3 bools the first is true if we recieved an update from tx,
        the second is true if we recieved an update from the target server, the third is true 
        if we recieved an update from the controller.
        """
        update_status = [False, False, False]
        active_sockets = {}
        try:
            active_sockets = dict(self.poller.poll(timeout=100))
        except:
            rospy.logfatal("RX{}: {}".format(self.rx_id, traceback.format_exc()))

        if self.sockets['receiver'] in active_sockets:
            # Called when socket is ready for reading, so non-blocking recv can be used.
            update = TxUpdate.from_bytes(self.sockets['receiver'].recv(flags=zmq.NOBLOCK))
            # If this is the first time we recieve an update we don't want to use the multilateraion pos
            # Instead get the target position directly from the target pos server and position the drone
            # from that.
            if self.desired_GPS is not None:
                self.desired_GPS = update_loc(update.target_coords, self.rx_id, self.is_local, self.get_rx_GPS())
            self.tx_state = update.states[0]
            self.tx_GPS = update.tx_GPS
            self.tx_timeout = time.time()
            update_status[0] = True

        if self.sockets['target'] in active_sockets:
            self.target_GPS = GPSCoord(*struct.unpack("ddd", self.sockets['target'].recv()))
            # Initialise the variable storing the Rx UAV's current posision relative to the initial target position
            if self.desired_GPS is None:
                self.desired_GPS = update_loc(self.target_GPS, self.rx_id, self.is_local)
            # The Rx UAV needs to know the Tx's UAV location to emulate the range readings. 
            # Get the first range reading by assuming the TX is above the target.
            if self.tx_GPS is None:
                self.tx_GPS = update_loc(self.target_GPS, TX_ID, self.is_local)
            update_status[1] = True
            self.target_timeout = time.time()


        # The controller is currently commented out as it is not finished yet
        # TODO: Implement the controller (which sends an update number to each drone to sync them)

        # if self.sockets['controller'] in active_sockets:
        #     update = self.sockets['controller'].recv(flags=zmq.NOBLOCK)

        #     self.reading_number = ContUpdate.from_bytes(update).update_num # The RXs should get their reading number which they send to the TX from the controller
        #     update_status[2] = True
        #     # rospy.logwarn('RX{}: Recived cont update!'.format(self.rx_id))
        #     self.controller_timeout = time.time()

        return update_status

    def send_update(self):
        """
        Sends an update to the TX drone containing this drones position, range reading, state,
        and the target's position (for error checking). Returns True if successful
        """
        # Calculate the emulated range based on the Rx, Tx and target positions.
        if self.target_GPS is not None:
            update = RxUpdate(
                rx_id = self.rx_id, 
                state = self.state, 
                timestamp = time.time(),
                reading_number = self.reading_number,
                rx_coords = self.get_rx_GPS(),
                range_reading = self.calculate_range(),
                target_coords = self.target_GPS)

            self.reading_number += 1
            self.sockets['sender'].send(update.to_bytes())

    def move_to_pos(self):
        """
        Command the drone to move to the desired GPS position stored in the attribute desired_GPS. 
        Check whether any failsafes are triggered before moving, if so return False.
        """
        real_val = True
        # See if the move falisafe is triggered
        if self.failsafes.move(self.current_GPS) == False:
            real_val = False

        # Only move if the failsafes are satisfied
        if real_val:
            self.mavros_controller.reach_position(
                        self.desired_GPS.lat,
                        self.desired_GPS.long,
                        self.desired_GPS.alt)

        # If the failsafes are not satisfied hold positon
        if not real_val:
            self.mavros_controller.reach_position(
                        self.current_GPS.lat,
                        self.current_GPS.long,
                        self.current_GPS.alt)

        return True



    def check_timeouts(self):
        """
        Returns true if a timeout has occoured 
        TODO: Implement this function. Timeouts are currently not checked
        """
        
        if self.tx_timeout is not None:
            
            if time.time() - self.tx_timeout > self.fp["COMMON"]["RX_TIMEOUT"]:
                return True

        if self.controller_timeout is not None:
            if time.time() - self.controller_timeout > self.fp["COMMON"]["CONTROLLER_TIMEOUT"]:
                return True

        if self.target_timeout is not None:
            rospy.logwarn("RX{} timeout: {}".format(self.rx_id, time.time() - self.target_timeout))
            if time.time() - self.target_timeout > self.fp["COMMON"]["TARGET_POS_TIMEOUT"]:
                return True
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'rx_id',
        type=int,
        choices=range(1, NUM_RXS + 1),
        help='the ID of this UAV (Rx UAVs should be numbered starting from 1)')
    parser.add_argument(
        '-f',
        '--field',
        action='store_true',
        help='used when running rx.py on a real UAV for a field test')
    parser.add_argument(
        '-l',
        '--local',
        action='store_true',
        help='specify to use the local flight parameters file')

    args = parser.parse_args()

    context = zmq.Context()
    start = time.time()
    # When running a simulation or practical flight test, perform UAV setup
    # before everything else so that the UAV is ready to fly.
    mavros_controller = MultiMavrosOffboardPosctl()
    mavros_controller.set_up(args.rx_id, args.field, args.local)
    if args.field:
        mavros_controller.setup_practical()
    else:
        mavros_controller.take_off()

    rx = ReceiverUAV(context, args.rx_id, mavros_controller, args.local)
    threshold_m = 1
    num_attempts = 0

    # ==============================================================================
    # Main Loop & RX State Machine
    # ==============================================================================
    send_time = time.time()
    log_time = time.time()

    last_state = rx.state
    loop_num = 0
    while True:
        try:
            # If the simulation is running, check for a rospy exit.
            if mavros_controller.is_shutdown():
                break
            
            updates = [False,False,False]

            # Once connected, check for updates and send updates every cycle
            if rx.state >= CONECTED_STATE:
                updates = rx.receive_updates()

        
            # if the drone is switched to offboard switch to the hold state
            if rx.state >=OFFBOARD_STATE:
                if rx.mavros_controller.state.mode != "OFFBOARD":
                    rx.state = HOLD_STATE
                

            if rx.state == ON_STATE:
                # As soon as RX turns on set up the sockets for communicating with RXs and controller
                if rx.set_up_conections():
                    rospy.loginfo("RX{}: Waiting for target server and controller".format(rx.rx_id))
                    rx.state = CONECTED_STATE
                else:
                    time.sleep(1) # Try again one second later
                    if num_attempts > 10: # If we have tried more than 10 times move the drones to the hold state 
                        rx.state = HOLD_STATE
                        rospy.logwarn("RX{}: maximum number of connection attempts reached".format(rx.rx_id))
                    num_attempts +=1


            elif rx.state == CONECTED_STATE:
                # Desired_GPS and reading_number not none only if we have recieved an update from the controller and TX
                if (rx.desired_GPS != None) and (rx.reading_number != None):
                    # This is run to make sure all the drones have the same update number. When the first GPS coord is recieved all the drones
                    # reset their update number.
                    # TODO: If a controller is implemented it is not necessary as the controller will send the same update number to all the drones 
                    rx.reading_number = 0
                    # Alert the user as to how far the drone is out of postion
                    p_str = "RX{} is out of position. Move it {:.1f}m North and {:.1f}m East. Press 'y' to continue. Press anything else for another update\n"
                    current_pos = rx.get_rx_GPS()
                    desired_pos = rx.desired_GPS
                    response = input(p_str.format(
                        rx.rx_id,
                        current_pos.y_distance(desired_pos),
                        current_pos.x_distance(desired_pos),
                    ))
                    # If the user responds with 'y' they have (hopefully) moved the drone close to its intended position so move to the next state
                    if response == 'y':
                        rospy.loginfo("RX{}: Waiting for offboard...".format(rx.rx_id))
                        rx.state = OFFBOARD_WAIT_STATE
   

            elif rx.state == OFFBOARD_WAIT_STATE:
                # Return true if the controller is in offboard mode
                # TODO: edit wait_for_offboard so it doesnt hold and just returns whether the drone is in offboard or not
                if mavros_controller.wait_for_offboard():
                    input("RX{}: Offboard activated. Move to formation start position?\n".format(rx.rx_id))
                    start = time.time() # Record the time to compare with later so we don't flood the terminal with prints
                    rx.state = OFFBOARD_STATE

            elif rx.state == OFFBOARD_STATE:
                
                current_pos = rx.get_rx_GPS()
                desired_pos = rx.desired_GPS

                if current_pos.distance(desired_pos) <= threshold_m:
                    rospy.loginfo("RX{}: In position. Waiting for TX".format(rx.rx_id))
                    # Tell the Tx that this Rx is ready
                    rx.state = READY_STATE
                elif abs(time.time() - start) > 1:
                    start = time.time()
                    # Move the drone to the starting position
                    mavros_controller.reach_position(
                        desired_pos.lat,
                        desired_pos.long,
                        desired_pos.alt) 
            

            elif rx.state == READY_STATE:
                if rx.tx_state == TRACKING_STATE: # If the TX is in tracking state also move to the tracking state
                    rx.state = TRACKING_STATE


            elif rx.state == TRACKING_STATE:

                if rx.tx_state == HOLD_STATE:
                    rx.state = HOLD_STATE

                # If we have not recieved an update from each server in the set time move the swarm to the hold state
                # TODO: Timeouts not currently working need to do controller first
                # if rx.check_timeouts():
                #     rx.state = HOLD_STATE
                
                # Try to move to the desired position and move to the hold state if a failsafe is triggered
                if not rx.move_to_pos():
                    rx.state = HOLD_STATE

            elif rx.state == HOLD_STATE:
                if rx.state is not last_state:
                    rospy.logwarn("RX{} Holding".format(rx.rx_id))
                    
            # Keep track of the last state so we can see if the state has changed
            last_state = rx.state
            loop_num += 1
            
            # Log items after everything has been updated and send update to tx
            if rx.state >= CONECTED_STATE:
                # If we have recieved an update from the target send an update to the tx
                if updates[1] or (abs(time.time() - send_time) > 1):
                    send_time = time.time()
                    # rospy.logwarn('RX{}: sending update to tx'.format(rx.rx_id))
                    rx.send_update()

                if (abs(time.time() - log_time) > rx.fp['COMMON']['LOG_FREQUENCY']):
                    log_time = time.time()
                    log_items = [
                        str(state_strs[rx.state]),              # state
                        str(rx.mavros_controller.state.mode),   # mode
                        str(rx.desired_GPS),                    # desired_GPS
                        str(rx.get_rx_GPS()),                   # current_GPS
                        str(rx.target_GPS),
                        str(rx.reading_number)
                        ]
                    rx.logger.log_items(log_items)
 
        except Exception as e:
            # Send one last update to all the other drones to let them know this one has failed
            rx.send_update()
            rospy.logfatal("UAV{}: {}".format(rx.rx_id, traceback.format_exc()))
            # Clean things up
            del mavros_controller
            rx.close_sockets()
            del rx
            break
    
    return False



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("TX: Interrupted.")
    except Exception as e:
        rospy.logfatal(traceback.format_exc())
    finally:
        sys.exit(0)
