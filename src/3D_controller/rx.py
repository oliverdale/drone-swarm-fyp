# External Libraries
import sys
import rospy
import traceback
import time
import argparse
import zmq
# Our stuff
import logger
from common import *
from gps import GPSCoord
from packets import ContUpdate, RxUpdate, TxUpdate, rxStateUpdate
from mavros_offboard_posctl import MultiMavrosOffboardPosctl

class ReceiverUAV:
    def __init__(self, context, rx_id, mavros_controller):

        # ID of this Rx, in the range 1 to NUM_RXS.
        self.context = context
        self.rx_id = rx_id
        self.mavros_controller = mavros_controller
        
        self.state = ON_STATE
        self.controller_state = None
        self.reading_number = 0 # To synchronize readings between drones 

        # These are recieved from the tx drone or the target server
        self.tx_state = None
        self.tx_GPS = None
        self.target_GPS = None
        self.desired_GPS = None
        self.current_GPS = None

        self.tx_timeout = None
        self.target_timeout = None
        self.controller_timeout = None
        
        # test var
        self.index_tester = 0

        # Get the ports, IP address and other constants from the json file
        fp = get_parameters() # Flight parameters
        self.fp = fp

        # Extract the connection info from the json
        self.sockets = {'controller':None, 'target':None, 'sender':None, 'receiver':None}
        self.controller_state_socket = None
        self.ips = {
            'controller':fp['COMMON']['CONTROLLER_IP_ADDRESS'],
            'target':fp['COMMON']['TARGET_POS_IP_ADDRESS'],
            'sender':fp['0']['IP_ADDRESS'], 
            'receiver':fp['0']['IP_ADDRESS']}

        self.ports = {
            'controller':fp['COMMON']['CONTROLLER_PORT'],
            'target':fp['COMMON']['TARGET_POS_PORT'],
            'sender':fp['COMMON']['RECEIVE_PORT'], # Sending to the receive port on the TX
            'receiver':fp['COMMON']['SEND_PORT']}  # Receiveing from the send port on the TX

        self.conn_info = {
            'controller':(zmq.SUB,zmq.SUBSCRIBE, b""),
            'target':(zmq.SUB,zmq.SUBSCRIBE, b""),
            'sender':(zmq.PUSH, zmq.LINGER, 0), 
            'receiver':(zmq.SUB,zmq.SUBSCRIBE, b"")}
        
        # Set up the logger
        items_to_log = ['state', 'mode', 'desired_GPS', 'current_GPS']
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

        try:
            # Creating pub socket to send states to controller
            self.controller_state_socket = self.context.socket(zmq.PUSH)
            self.controller_state_socket.setsockopt(zmq.LINGER, 0)
            self.controller_state_socket.connect("tcp://127.0.0.1:5559")
        except Exception as e:
            rospy.logfatal("RX{}: Could not setup controller sending connection".format(self.rx_id))
            rospy.logfatal(traceback.format_exc())
            success = False

        if success:
            # Poller to simultaneously wait for Tx updates and target coordinates.
            self.poller = zmq.Poller()
            self.poller.register(self.sockets['receiver'], zmq.POLLIN)
            self.poller.register(self.sockets['target'], zmq.POLLIN)
            self.poller.register(self.sockets['controller'], zmq.POLLIN)
        else:
            rospy.logfatal("RX{}: Could not setup sockets".format(self.rx_id))
            self.close_sockets()

        return success

    def close_sockets(self):
        """
        Make sure all the sockets are closed when RX terminates
        """
        for socket in self.sockets.values():
            if socket is not None:
                socket.close()
        if self.controller_state_socket is not None:
            self.controller_state_socket.close()
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
            active_sockets = dict(self.poller.poll(timeout=min(2, 3)))
        except:
            rospy.logfatal("RX{}: {}".format(self.rx_id, traceback.format_exc()))
            self.close_sockets()

        if self.sockets['receiver'] in active_sockets:
            # Called when socket is ready for reading, so non-blocking recv can be used.
            update = TxUpdate.from_bytes(self.sockets['receiver'].recv(flags=zmq.NOBLOCK))
            # If this is the first time we recieve an update we don't want to use the multilateraion pos
            # Instead get the target position directly from the target pos server and position the drone
            # from that.
            if self.desired_GPS is not None:
                self.desired_GPS = update_loc(update.target_coords, self.rx_id, self.get_rx_GPS())
            self.tx_state = update.states[0]
            self.tx_GPS = update.tx_GPS
            self.tx_timeout = time.time()
            update_status[0] = True

        if self.sockets['target'] in active_sockets:
            # rospy.logwarn('RX{} got target pos packet'.format(self.rx_id))
            self.target_GPS = GPSCoord(*struct.unpack("ddd", self.sockets['target'].recv()))
            # Initialise the variable storing the Rx UAV's current posision relative to the initial target position
            if self.desired_GPS is None:
                self.desired_GPS = update_loc(self.target_GPS, self.rx_id)
            # The Rx UAV needs to know the Tx's UAV location to emulate the range readings. 
            # Get the first range reading by assuming the TX is above the target.
            if self.tx_GPS is None:
                self.tx_GPS = update_loc(self.target_GPS, TX_ID)
            update_status[1] = True
            self.target_timeout = time.time()

        if self.sockets['controller'] in active_sockets:
            update = ContUpdate.from_bytes(self.sockets['controller'].recv(flags=zmq.NOBLOCK))
            # rospy.logwarn(update)
            self.reading_number = update.update_num # The RXs should get their reading number which they send to the TX from the controller
            self.controller_state =update.commands[self.rx_id]
            update_status[2] = True
            #rospy.logwarn('RX{}: Recived cont update!'.format(self.rx_id))
            self.controller_timeout = time.time()

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
                # seq_no = self.reading_number,
                seq_no = self.index_tester,
                rx_coords = self.get_rx_GPS(),
                range_reading = -1 if self.target_GPS is None else self.calculate_range(),
                target_coords = GPSCoord(-1,-1,-1) if self.target_GPS is None else self.target_GPS)
            self.index_tester += 1
            self.reading_number += 1
            # rospy.logwarn("RX:{} Sent rxupdate #{}".format(self.rx_id,self.index_tester))
            self.sockets['sender'].send(update.to_bytes())

    def send_state_update(self):
        """
        Sends update of RX state to controller
        """
        # rospy.logwarn('RX{}: Sent cont stat update'.format(self.rx_id))
        rx_coords = self.get_rx_GPS()
        target_coords = GPSCoord(-1,-1,-1) if self.target_GPS is None else self.target_GPS
        update = rxStateUpdate(self.rx_id, self.state, rx_coords, target_coords)
        self.controller_state_socket.send(update.to_bytes())
        # rospy.logwarn(update)

    def move_to_pos(self):
        """
        Command the drone to move to the desired GPS position stored in the attribute desired_GPS"""
        self.mavros_controller.reach_position(
                    self.desired_GPS.lat,
                    self.desired_GPS.long)

    def check_timeouts(self):
        """
        Returns true if a timeout has occoured
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

    rx = ReceiverUAV(context, args.rx_id, mavros_controller)
    threshold_m = 1
    num_attempts = 0

    # ==============================================================================
    # Main Loop & RX State Machine
    # ==============================================================================
    send_time = time.time()

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
                # If we have recieved an update from the controller send an update to the tx
                if updates[2] or (abs(time.time() - send_time) > 1):
                    send_time = time.time()
                    # rospy.logwarn('RX{}: sending update to tx'.format(rx.rx_id))
                    rx.send_update()
                    rx.send_state_update()
           
                    log_items = [
                        str(state_strs[rx.state]),              # state
                        str(rx.mavros_controller.state.mode),   # mode
                        str(rx.desired_GPS),                    # desired_GPS
                        str(rx.get_rx_GPS())]                   # current_GPS
                     
                    rx.logger.log_items(log_items)


            if rx.state == ON_STATE:
                # As soon as TX turns on set up the sockets for communicating with RXs and controller
                if rx.set_up_conections():
                    rospy.loginfo("RX{}: Waiting for target server and controller".format(rx.rx_id))
                    rx.state = CONECTED_STATE
                else:
                    time.sleep(1)
                    if num_attempts > 10:
                        rx.state = HOLD_STATE
                        rospy.logwarn("RX{}: maximum number of connection attempts reached".format(rx.rx_id))
                    num_attempts +=1


            elif rx.state == CONECTED_STATE:
                if (rx.desired_GPS != None) and (rx.reading_number != None):
                    p_str = "RX{} is out of position. Move it {:.1f}m North and {:.1f}m East. Press 'y' to continue. Press anything else for another update\n"
                    current_pos = rx.get_rx_GPS()
                    desired_pos = rx.desired_GPS
                    # response = input(p_str.format(
                    #     rx.rx_id,
                    #     current_pos.y_distance(desired_pos),
                    #     current_pos.x_distance(desired_pos),
                    # ))
                    # if response == 'y':
                    if rx.controller_state == 2:
                        rospy.loginfo("RX{}: Waiting for offboard...".format(rx.rx_id))
                        rx.state = OFFBOARD_WAIT_STATE
   

            elif rx.state == OFFBOARD_WAIT_STATE:
                if mavros_controller.wait_for_offboard():
                    # input("RX{}: Offboard activated. Move to formation start position?\n".format(rx.rx_id))
                    if (rx.controller_state == 3):
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
                    mavros_controller.reach_position(
                        desired_pos.lat,
                        desired_pos.long,
                        desired_pos.alt)
            

            elif rx.state == READY_STATE:
                if rx.tx_state == TRACKING_STATE:
                    rx.state = TRACKING_STATE


            elif rx.state == TRACKING_STATE:

                if rx.tx_state == HOLD_STATE:
                    rx.state = HOLD_STATE

                # If we have not recieved an update from each server in the set time move the swarm to the hold state
                # TODO: Timeouts not currently working need to do controller first
                # if rx.check_timeouts():
                #     rx.state = HOLD_STATE

                rx.mavros_controller.reach_position(
                    rx.desired_GPS.lat,
                    rx.desired_GPS.long)

            elif rx.state == HOLD_STATE:
                if rx.state is not last_state:
                    rospy.logwarn("RX{} Holding".format(rx.rx_id))
                    
            
            last_state = rx.state
            loop_num += 1
        
 
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("TX: Interrupted.")
    except Exception as e:
        rospy.logfatal(traceback.format_exc())
    finally:
        sys.exit(0)
