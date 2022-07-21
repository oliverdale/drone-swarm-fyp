"""
filename: tx.py
author: Rowan Sinclair
date: 30th September 2021
description:
    The main file which is run on the TX drone. Contains the logic for sending and reciving updates 
    and the state machine which controls the drone. Also performs the multilateration and
    Kalman filtering
"""

# External Libraries
from math import fabs
import sys
from tkinter.constants import N

from zmq.sugar.context import T
import rospy
import traceback
import time
import argparse
import zmq
# Our stuff
from failsafes import Failsafes
import logger
from common import *
from packets import ContUpdate, RxUpdate, TxUpdate
from mavros_offboard_posctl import MultiMavrosOffboardPosctl
from update_store import UpdateStore
import multilateration


class TransmitterUAV:
    def __init__(self, context, mavros_controller, is_local=False):

        self.context = context
        self.id = 0 # ID of tx is 0
        self.mavros_controller = mavros_controller
        self.is_local = is_local
        
        self.state = ON_STATE
        self.rx_states = [ON_STATE]*4
        self.reading_number = 0 # To synchronize readings between drones 

        # Store all the updates received from the Rxs.
        self.updates = UpdateStore()
        
        self.act_target_GPS = None # This is recieved from the rx drones
        self.est_target_GPS = None # This is estimated from the ranges
        self.desired_GPS = None
        self.current_GPS = None

        self.pos_filter = None

        self.controller_timeout = None
        self.rx_timeout = None


        # Get the ports, IP address and other constants from the json file
        fp = get_parameters(is_local) # Flight parameters
        self.fp = fp
        self.failsafes = Failsafes(is_local)

        # Extract the connection info from the json
        self.sockets = {'sender':None, 'receiver':None} # , 'controller':None
        self.ips = {
            # 'controller':fp['COMMON']['CONTROLLER_IP_ADDRESS'],
            'sender':None, 
            'receiver':None}

        self.ports = {
            # 'controller':fp['COMMON']['CONTROLLER_PORT'],
            'sender':fp['COMMON']['SEND_PORT'], 
            'receiver':fp['COMMON']['RECEIVE_PORT']}

        self.conn_info = {
            # 'controller':(zmq.SUB,zmq.SUBSCRIBE, b""),
            'sender':(zmq.PUB, None, None), 
            'receiver':(zmq.PULL,None, None)}
        
        # Set up the logger
        items_to_log = [
            'states_list',
            'target_filt_CART',
            'target_filt_GPS',
            'target_real_GPS',
            'tx_filt_CART',
            'tx_filt_GPS',
            'tx_real_GPS',
            'multilateration_error',
            'mode',
            'reading_number']
        self.logger = logger.Logger(0, items_to_log)


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
                if self.conn_info[key][1] is not None:
                    self.sockets[key].setsockopt(self.conn_info[key][1], self.conn_info[key][2])
                if self.ips[key] is None: # If no IP address is supplied set up the connection on the local host
                    self.sockets[key].bind("tcp://*:{}".format(self.ports[key]))
                else:
                    self.sockets[key].connect("tcp://{}:{}".format(self.ips[key],self.ports[key]))
            except Exception as e:
                rospy.logfatal("TX{}: Could not setup {} connection".format(self.id, key))
                rospy.logfatal(traceback.format_exc())
                success = False

        if success:
            # Poller to simultaneously wait for Tx updates and target coordinates.
            self.poller = zmq.Poller()
            self.poller.register(self.sockets['receiver'], zmq.POLLIN)
            # self.poller.register(self.sockets['controller'], zmq.POLLIN)
        else:
            rospy.logfatal("TX{}: Could not setup sockets".format(self.id))
            success = False

        return success

    def close_sockets(self):
        """
        Make sure all the sockets are closed when RX terminates
        """
        for socket in self.sockets.values():
            if socket is not None:
                socket.close()

        rospy.logwarn("TX{}: Closed sockets".format(self.id))

    def get_tx_GPS(self):
        """
        Returns the current GPS position of the Tx drone and sets the attribute
        """
        self.current_GPS = self.mavros_controller.get_current_position()
        return self.current_GPS

    
    def receive_updates(self):
        """
        Checks for updates on all the sockets and updates the associated attributes.
        Returns a list of 2 bools the first is true if we recieved an update from all rxs,
        the second is true if we recieved an update from the controller.
        """
        update_status = [False, False]
        active_sockets = {}
        try:
            active_sockets = dict(self.poller.poll(timeout=100))
        except:
            rospy.logfatal("RX{}: {}".format(self.rx_id, traceback.format_exc()))

        if self.sockets['receiver'] in active_sockets:
            message = self.sockets['receiver'].recv(flags=zmq.NOBLOCK)
            update = RxUpdate.from_bytes(message)
            self.act_target_GPS = update.target_coords
        
            # If this is the first time through the loop set these so the drone knows where to go
            if self.est_target_GPS is None:
                self.est_target_GPS = update.target_coords
                self.desired_GPS = update_loc(self.act_target_GPS, TX_ID, self.is_local)

            # Check if we have a full set of updates from all the rxs
            if self.updates.store(update):
                update_status[0] = True
                self.rx_states = self.updates.get_states()
                self.rx_timeout = time.time()
                # Not needed if we have a controller as the reading number is sent from it.
                self.reading_number = int(self.updates.current_seq_key)

        # The controller is currently commented out as it is not finished yet
        # TODO: Implement the controller (which sends an update number to each drone to sync them) 

        # if self.sockets['controller'] in active_sockets:
        #     message = self.sockets['controller'].recv(flags=zmq.NOBLOCK)
        #     update = ContUpdate.from_bytes(message)
        #     self.reading_number = update.update_num # TODO compare the reading number from the controller to those recieved from the RXs to make sure everything is in sync
        #     update_status[1] = True
        #     self.controller_timeout = time.time()

        return update_status

    def perform_multilateration(self):
        """ 
        Updates the est_target_GPS attribute using the multilateration posfilter.
        """
        rx_positions = self.updates.get_rx_positions() # Get the locations of the rx drones
        ranges = self.updates.get_ranges() # Calculate the simulated radar ranges
        drones = [self.get_tx_GPS()] + rx_positions
        # Perform multilateration and update the rx attributes accordingly
        self.est_target_GPS = self.pos_filter.est_target_pos(drones, ranges, 1)

    def pre_run_init(self):
        """
        Initialise the multilateration filter before tracking starts
        """
        drone_positions = [self.get_tx_GPS()] + self.updates.get_rx_positions()

        self.pos_filter = multilateration.PosFilter(
            drones_i = drone_positions,
            z_std_drone = self.fp["COMMON"]["Z_STD_DRONE"],
            z_std_range = self.fp["COMMON"]["Z_STD_RANGE"],
            initial_target = self.act_target_GPS,
            initial_target_uncertanty = 0.1,
            is_local = self.is_local)
    

    def send_update(self):
        """
        Sends an update to the RX drones containing the target's estimated position,
        the states of all the drones and this drones position. Returns True if successful
        """
        # Send the desired centre position and the Tx position to the Rxs.
        states_list = [self.state] + self.updates.get_states()
        # Before tracking starts there est_target_GPS is equal to act_target_GPS
        update = TxUpdate(states_list, self.est_target_GPS, self.get_tx_GPS())
        self.sockets['sender'].send(update.to_bytes())

    def move_to_pos(self):
        """
        Command the drone to move to the desired GPS position stored in the attribute desired_GPS. 
        Check whether any failsafes are triggered before moving, if so return False.
        """
        return_val = True
        real_val = True
        if self.failsafes.overlaps(self.get_tx_GPS(), self.updates.get_rx_positions()) == False:
            return_val = False
        if self.failsafes.positions(self.get_tx_GPS(), self.updates.get_rx_positions()) == False:
            return_val = False
        if self.failsafes.move(self.current_GPS) == False:
            real_val = False

        # Only move if the failsafes are satisfied
        if return_val and real_val:
            self.mavros_controller.reach_position(
                        self.desired_GPS.lat,
                        self.desired_GPS.long,
                        self.desired_GPS.alt)
        
        if not real_val:
            self.mavros_controller.reach_position(
                        self.current_GPS.lat,
                        self.current_GPS.long,
                        self.current_GPS.alt)

        return return_val

    def check_timeouts(self):
        """
        Returns true if a timeout has occoured
        """
        rospy.logwarn(str("rx_timeout: ") + str(self.rx_timeout))
        if self.rx_timeout is not None:
            if time.time() - self.rx_timeout > self.fp["COMMON"]["RX_TIMEOUT"]:
                return True
        if self.controller_timeout is not None:
            if time.time() - self.controller_timeout > self.fp["COMMON"]["CONTROLLER_TIMEOUT"]:
                return True
        return False



def main():
    parser = argparse.ArgumentParser()
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
    mavros_controller.set_up(TX_ID, args.field, args.local)
    if args.field:
        mavros_controller.setup_practical()
    else:
        mavros_controller.take_off()

    tx = TransmitterUAV(context, mavros_controller, args.local)
    threshold_m = 1
    num_attempts = 0
    log_time = time.time()

    # ==============================================================================
    # Main Loop & RX State Machine
    # ==============================================================================
    last_state = tx.state
    loop_num = 0
    while True:
        try:
            # If the simulation is running, check for a rospy exit.
            if mavros_controller.is_shutdown():
                break
            
            updates = [False,False]
            # Once connected, check for updates and send updates every cycle
            if tx.state >= CONECTED_STATE:
                tx.get_tx_GPS()
                updates = tx.receive_updates()
                # Update the Rx drones if we recieve an update from all them containing the target's location
              

            # If the drone is switched to offboard switch to the hold state
            if tx.state >=OFFBOARD_STATE:
                if tx.mavros_controller.state.mode != "OFFBOARD":
                    tx.state = HOLD_STATE
                
                # If any of the rx drones switch to hold state also switch tx
                if any(state == HOLD_STATE for state in tx.rx_states):
                    tx.state == HOLD_STATE

                # If we have not recieved an update from each server in the set time move the swarm to the hold state
                # TODO: timeouts not currently working, do controller first
                # if tx.check_timeouts():
                #     tx.state = HOLD_STATE

            if tx.state == ON_STATE:
                # As soon as TX turns on set up the sockets for communicating with RXs and controller
                if tx.set_up_conections():
                    rospy.loginfo("TX{}: Waiting for target and controller server".format(tx.id))
                    tx.state = CONECTED_STATE
                else:
                    time.sleep(1) # Try again one second later
                    if num_attempts > 10: # If we have tried more than 10 times move the drones to the hold state 
                        tx.state = HOLD_STATE
                        rospy.logwarn("TX: maximum number of connection attempts reached")
                    num_attempts +=1


            elif tx.state == CONECTED_STATE:
                # rospy.logwarn(str(tx.desired_GPS ))
                # Desired_GPS and reading_number not none only if we have recieved an update from the controller and TX
                if (tx.desired_GPS != None) and (tx.reading_number != None):
                    # This is run to reset the update class and make sure all the drones have the same update number.
                    # When the first GPS coord is recieved all the drones reset their update number.
                    # TODO: If a controller is implemented it is not necessary as the controller will send the same update number to all the drones 
                    tx.updates.reset()
                    tx.reading_number = 0
                    # Alert the user as to how far the drone is out of postion
                    p_str = "TX{} is out of position. Move it {:.1f}m North and {:.1f}m East. Press 'y' to continue. Press anything else for another update\n"
                    current_pos = tx.get_tx_GPS()
                    desired_pos = tx.desired_GPS
                    response = input(p_str.format(
                        tx.id,
                        current_pos.y_distance(desired_pos),
                        current_pos.x_distance(desired_pos),
                    ))
                    # If the user responds with 'y' they have (hopefully) moved the drone close to its intended position so move to the next state
                    if response == 'y':
                        rospy.loginfo("TX{}: Waiting for offboard...".format(tx.id))
                        tx.state = OFFBOARD_WAIT_STATE
   

            elif tx.state == OFFBOARD_WAIT_STATE:
                if mavros_controller.wait_for_offboard():
                    input("TX{}: Offboard activated. Move to formation start position?\n".format(tx.id))
                    start = time.time() # Record the time to compare with later so we don't flood the terminal with prints
                    tx.state = OFFBOARD_STATE

            elif tx.state == OFFBOARD_STATE:
                current_pos = tx.get_tx_GPS()
                desired_pos = tx.desired_GPS

                if current_pos.distance(desired_pos) <= threshold_m:
                    rospy.loginfo("TX{}: In position. Waiting for TX".format(tx.id))
                    # Tell the Tx that this Rx is ready
                    tx.state = READY_STATE
                elif abs(time.time() - start) > 1:
                    start = time.time()
                    mavros_controller.reach_position(
                        desired_pos.lat,
                        desired_pos.long,
                        desired_pos.alt)
            
            elif tx.state == READY_STATE:
                
                if all(state == READY_STATE for state in tx.rx_states):
                    input('All drones ready. Start Tracking?')
                    
                    tx.state = TRACKING_STATE

            elif tx.state == TRACKING_STATE:
                rospy.loginfo(str(tx.rx_states) + ', '+  str(tx.state))

                # Start tracking once all the drones are in the tracking state
                if all(state == TRACKING_STATE for state in tx.rx_states):
                    rospy.loginfo(str(tx.act_target_GPS) + ', ' + str(tx.est_target_GPS))
                    if tx.pos_filter is None:
                        tx.pre_run_init()
                        
                    tx.perform_multilateration()
                    tx.desired_GPS = update_loc(tx.est_target_GPS, TX_ID, tx.is_local) # Change est to act to remove multilateration/kalman

                    # Try to move to the desired position and move to the hold state if a failsafe is triggered
                    if not tx.move_to_pos():
                        tx.state == HOLD_STATE
                
            
            elif tx.state == HOLD_STATE:
                if last_state is not tx.state:
                    rospy.logfatal("The swarm has entered a hold state due to an error")
  

            # Update and log after all the variables have changed
            if tx.state >= CONECTED_STATE:
                # Update the Rx drones if we recieve an update from all them containing the target's location
                if updates[0]:    
                    tx.send_update()
                # Log every Log peroid
                if (abs(time.time() - log_time) > tx.fp['COMMON']['LOG_FREQUENCY']):
                    log_time = time.time()
                    log_items = [
                        str(tx.rx_states)+str(tx.state),
                        str(None if tx.pos_filter is None else tx.pos_filter.get_swarm_item_pos(5)[-1, :]),         # cart_target_filt
                        str(None if tx.est_target_GPS is None else tx.est_target_GPS),                              # gps_target_filt
                        str(None if tx.act_target_GPS is None else tx.act_target_GPS),                              # gps_target_real
                        str(None if tx.pos_filter is None else tx.pos_filter.get_swarm_item_pos(0)[-1, :]),         # cart_tx_filt
                        str(tx.desired_GPS),                                                                        # gps_tx_filt
                        str(tx.current_GPS),                                                                        # gps_tx_real
                        str(-1 if tx.est_target_GPS is None else tx.est_target_GPS.distance3d(tx.act_target_GPS)),  # multilateration_error
                        str(tx.mavros_controller.state.mode),                                                       # mode
                        str(tx.reading_number)
                    ]
                    tx.logger.log_items(log_items)

            # Updates after each loop
            last_state = tx.state
            loop_num += 1

        except Exception as e:
            # Send one last update to all the other drones to let them know this one has failed
            tx.send_update()
            rospy.logfatal("UAV0: {}".format(traceback.format_exc()))
            # Clean things up
            del mavros_controller
            tx.close_sockets()
            del tx
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
