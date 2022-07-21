import sys
import time
from zmq.sugar.constants import THREAD_PRIORITY_DFLT

from zmq.sugar.context import T
import rospy
import zmq
import traceback
from common import *
from packets import ContUpdate, RxUpdate, TxUpdate, rxStateUpdate
import tkinter as tk
from tkinter.constants import DISABLED, RIDGE
import threading as thread
import time
from gps import GPSCoord

class Controller:

    def __init__(self, context):
        self.context = context
        self.receiver_rx_soc = None
        self.receiver_tx_soc = None
        self.controller_soc = None
        self.receiver_rx_states_soc = None
        self.drone_GPSs = {}
        self.target_coords = None
        self.drone_states = [0]*5
        self.received_drone_states = {}
        self.received_state = None
        self.send_num = 0
        self.target_server_alive = False

        # Get the ports, IP address and other constants from the json file
        fp = get_parameters() # Flight parameters
        self.fp = fp

    def connect(self):
        """
            Create connections between controller, target server, rx, and tx 
            drones.
        """
        success = True
        '''problem: cant get the packets using .connect - using .bind
            conflicts with tx.py'''
        try:
            # Create a SUB socket to receive updates from the Tx.
            self.receiver_tx_soc = self.context.socket(zmq.SUB)
            self.receiver_tx_soc.connect("tcp://{}:{}".format(self.fp['0']['IP_ADDRESS'], self.fp['COMMON']['SEND_PORT']))
            self.receiver_tx_soc.setsockopt_string(zmq.SUBSCRIBE, "")
        except Exception as e:
            print("RX{}: Could not setup tx reciever connection".format(self.rx_id))
            success = False

        try:
            # Create a PUB socket to send updates to the tx and rx drones.
            self.controller_soc = self.context.socket(zmq.PUB)
            self.controller_soc.bind("tcp://*:{}".format(self.fp['COMMON']['CONTROLLER_PORT']))
        except Exception as e:
            print("RX{}: Could not setup controller connection".format(self.rx_id))
            success = False

        try:
            # Create PULL socket for getting rx state updates.
            self.state_soc = self.context.socket(zmq.PULL)
            self.state_soc.bind("tcp://*:5559")
        except Exception as e:
            print("CONNECTION WITH RX STATE PIPE FAILED")
            success = False

        try:
            # Create SUB socket for getting target positions.
            # Need to know when drones are ready for 'y' input.
            self.target_soc = self.context.socket(zmq.SUB)
            self.target_soc.setsockopt(zmq.SUBSCRIBE, b"")
            self.target_soc.connect("tcp://localhost:5553")
        except Exception as e:
            print("CONNECTION WITH TARGET SEVER FAILED")
            sucess = False
            
        if success:
            # Poller to simultaneously wait for Tx updates and target coordinates.
            self.poller = zmq.Poller()
            # self.poller.register(self.receiver_rx_soc, zmq.POLLIN)
            self.poller.register(self.receiver_tx_soc, zmq.POLLIN)
            self.poller.register(self.state_soc, zmq.POLLIN)
            self.poller.register(self.target_soc, zmq.POLLIN)
        else:
            self.close_sockets()

        return success

    def close_sockets(self):
        """
        Make sure all the sockets are closed when RX terminates
        """
        try:
            if self.receiver_rx_soc is not None:
                self.receiver_rx_soc.close()
            if self.receiver_tx_soc is not None:
                self.receiver_tx_soc.close()
            if self.controller_soc is not None:
                self.controller_soc.close()
            if self.state_soc is not None:
                self.state_soc.close()
            if self.target_soc is not None:
                self.target_soc.close()
        except:
            print("socket closing error")
        print("Sockets closed.")

    def send_update(self, commands):
        '''
            Perioically called to send update on state information for
            all tx and rx drones.    
        '''
        if self.target_server_alive:
            update = ContUpdate(self.send_num, commands)
            self.controller_soc.send(update.to_bytes())
            self.send_num += 1
            print(update)

    def receive_updates(self):
        """
        Checks for updates on all the sockets and updates the associated attributes.
        Returns a list of 2 bools the first is true if we recieved an update from all rxs,
        the second is true if we recieved an update from the tx.
        """
        update_status = [False, False]
        active_sockets = {}
        if (active_sockets != {}):
            print(active_sockets)
        active_sockets = dict(self.poller.poll(timeout=100))
        
        if self.receiver_tx_soc in active_sockets:
            message = self.receiver_tx_soc.recv(flags=zmq.NOBLOCK)
            # update = rxStateUpdate.from_bytes(message)
            # self.parse_update(update)
        
        if self.state_soc in active_sockets:
            message = self.state_soc.recv(flags=zmq.NOBLOCK)
            update =  rxStateUpdate.from_bytes(message)
            self.parse_update(update)
            self.receive_updates()
        
        if self.target_soc in active_sockets:
            self.target_server_alive = True
            self.target_soc.close()
        
        return update_status

    def parse_update(self, update):
        """ Takes an update from Rx indicating its current state
            and updates the local information.
        """
        # print(update)
        self.received_drone_states[update.rxid] = update.state
        self.drone_GPSs[update.rxid] = update.rx_coords
        if update.rxid > 0:
            # Only take target coord updates from rx drones.
            self.target_coords = update.target_coords

    def run(self, stop):
        '''
            Controller main loop.
            Responsible for receiving and sending updates periodically.
        '''
        self.connect()
        last_time = time.time()
        while True:
            try:
                if time.time() - last_time > self.fp["COMMON"]["UPDATE_PEROID"]:
                    self.send_update(self.drone_states)
                    self.receive_updates()
                    last_time = time.time()
            except KeyboardInterrupt:
                self.close_sockets()
                break    
            if stop():
                print("Controller thread killed.")
                break

        self.close_sockets()

class CONTROLLERGUI():
    def __init__(self, master, controller):
        self.master = master
        self.controller = controller

        self.position_check = False

        self.canvas = tk.Canvas(master, height=500, width=900, bg="#E4E4E4")
        self.canvas.pack()
        self.title = tk.Label(master, text="UAV Flight Controller", fg="black", font=('Helvetica', 12, 'bold'))
        self.title.place(relx=0, rely=0, relheight=0.05, relwidth=1)

        # Start button - begins new thread for controller running
        self.controller_thread_stop = False
        self.controller_thread = thread.Thread(target=self.controller.run, args =(lambda : self.controller_thread_stop, ))
        self.startButton = tk.Button(master, text="Start Controller", fg="black", command=self.start)
        self.startButton.place(relx=0,rely=0.05,relwidth=.2,relheight=.1)

        # Stop button - kills controller thread and closes Tkinter
        self.stopButton = tk.Button(master, text="Stop Controller", fg="black", command=self.stop)
        self.stopButton.place(relx=.8,rely=.9,relwidth=.2,relheight=.1)

        # Frames for each individual UAV
        self.tx_guis = {'navi0':TX_GUI(master, 0, self.controller)}
        self.tx_state_flag = 0
        self.uav_guis = {
            'navi1':DRONE_GUI(master, 1, self.controller),
            'navi2':DRONE_GUI(master, 2, self.controller),
            'navi3':DRONE_GUI(master, 3, self.controller),
            'navi4':DRONE_GUI(master, 4, self.controller)}
        self.rx_state_flags = {
            'navi1':0,
            'navi2':0,
            'navi3':0,
            'navi4':0}

    def start(self):
        '''
            Called when Start Controller button is pressed.
            Starts the controller main thread.
            Starts the gui main thread.
            Moves al drones into the connecting state.
        '''

        # Start controller thread
        self.controller_thread.start()

        self.startButton.config(state='disabled')

        # Move all drones into connecting state
        for uav_gui in self.uav_guis.items():
            uav_gui[1].connecting()
        self.tx_guis['navi0'].connecting()

        # Create button that enables all drones to be put in hold.
        self.holdButton = tk.Button(self.canvas, text='STOP DRONES', fg='white', bg='red', command=self.kill_drones)
        self.holdButton.place(relx=0, rely=0.2, relwidth=0.2, relheight=0.1)

        # Create label indicating that we are waiting for target server
        self.targetWaitingLabel = tk.Label(self.canvas, text='Waiting for target server...', bg='#E4E4E4')
        self.targetWaitingLabel.place(relx=0,rely=0.4,relwidth=0.2,relheight=.1)

        # Start gui main thread
        self.run()

        return None

    def run(self):
        '''
            gui main thread. Calls check state periodically (every 1s).
        '''
        # Check if target server connection is alive and if so destroy label
        if self.controller.target_server_alive:
            self.targetWaitingLabel.destroy()

        # Check states of the drones
        self.check_states()            

        # Call run() after 1000ms - non blocking
        self.master.after(1000, self.run)

        return None

    def kill_drones(self):
        '''
            Called if stop drones button is pressed.
            Will put the drones into hold state.
            
        '''
        for i in range(len(self.controller.drone_states)):
            self.controller.drone_states[i] = 5    # each drone is in hold state
        return None

    def check_states(self):
        '''
            Called periodically.
            Iterates through the states for rx and tx drones.
            Calls appropriate functions for corresponding drone state.
        '''

        # Iterates through controller owned received_drone_states
        for state in self.controller.received_drone_states.items():
            rx_id = state[0]
            rx_state = state[1]
            
            # Connected state
            if rx_state == 1:
                # Tx drones
                if rx_id == 0:
                    if self.tx_state_flag != 1:
                        self.tx_guis['navi{}'.format(rx_id)].connected()
                        self.controller.drone_states[rx_id] = 1
                        self.tx_state_flag = 1
                    self.tx_guis['navi{}'.format(rx_id)].position_check()
                # Rx drones
                elif rx_id >= 1:
                    if self.rx_state_flags['navi{}'.format(rx_id)] != 1:
                        self.uav_guis['navi{}'.format(rx_id)].connected()
                        self.controller.drone_states[rx_id] = 1
                        self.rx_state_flags['navi{}'.format(rx_id)] = 1
                    self.uav_guis['navi{}'.format(rx_id)].position_check()
            
            # Offboard wait state
            if rx_state >= 1:
                if rx_id == 0:
                    self.tx_guis['navi{}'.format(rx_id)].display_gps()
                elif rx_id >= 1:
                    self.uav_guis['navi{}'.format(rx_id)].display_gps()

            # Offboard state
            if rx_state == 2:
                # Tx drones
                if rx_id == 0:
                    if self.tx_state_flag != 2:
                        self.tx_guis['navi{}'.format(rx_id)].offboard_wait()
                        self.controller.drone_states[rx_id] = 2
                        self.tx_state_flag = 2
                # Rx drones
                elif rx_id >= 1:
                    if self.rx_state_flags['navi{}'.format(rx_id)] != 2:
                        self.uav_guis['navi{}'.format(rx_id)].offboard_wait()
                        self.controller.drone_states[rx_id] = 2
                        self.rx_state_flags['navi{}'.format(rx_id)] = 2
                
            # Tracking state
            if rx_state == 3:
                if rx_id == 0:
                    if self.tx_state_flag != 3:
                        self.tx_guis['navi{}'.format(rx_id)].track_prompt()
                        self.controller.drone_states[rx_id] = 3
                        self.tx_state_flag = 3

    def stop(self):
        '''
            Called when stop button is pressed.
            Stops main controller thread.
            Closes all sockets.
            Destroys gui.
        '''
        self.controller_thread_stop = True
        self.controller.close_sockets
        self.master.destroy()

class DRONE_GUI():
    '''
        DRONE_GUI class.
        Creates column in gui for an idividual drone.
        Provides prompts, text, and buttons for controlling behaviour of individual
        drones.
    '''

    def __init__(self, master, id, controller):
        ''' 
            Parameters:
            -----------
                master : Tk
                    Toplevel widget representing the main window of the gui.
                id : int
                    ID of the drone. Rx drones are 1-4, Tx drone is 0.
                controller : Controller
                    Controller used for main loop in communicating between target
                    positioning server, rx.py and tx.py instances.

            Attributes:
            -----------
                self.frame : tk.Frame
                    TKinter Frame widget responsible for arranging the position of
                    widgets belonging to the column which represents each drone.
                self.p_str : str
                    String template for displaying the distance of a drone from the
                    desired position of the drone.
        '''

        self.master = master
        self.id = id
        self.controller = controller

        # Creates column-like frame for controller actions specific to each drone.
        self.frame = tk.Frame(master, bg='white', borderwidth=2, relief=RIDGE)
        self.frame.place(relx=0.35+0.15*(id-1),rely=0.05,relwidth=0.15,relheight=0.8)

        self.p_str = "RX{} is out of position. Move it {:.1f}m North and {:.1f}m East."

        return None

    def connecting(self):
        ''' 
            Called when the start controller button is pressed.
            Creates connecting label for when drone is being initialised, indicating
            we are waiting for the drones to connect to the controller.
        '''

        # Create connecting label.
        self.connectingLabel = tk.Label(self.frame, text="connecting...", fg="grey")
        self.connectingLabel.place(relx=0.05,rely=0.05,relwidth=0.9,relheight=.1)

        return None

    def connected(self):
        ''' 
            Called when drone is connected.
            Fills in connecting label green to indicate connection for drone.
            Creates label indicating drones distance to the desired position.
            Creates button for to send update for when drones are in position.
        '''

        # Fill in connecting label green.
        self.connectingLabel.config(text='Connected', bg='green', fg='white')

        # Create in position button.
        self.inPositionButton = tk.Button(self.frame, text='In position?' ,command=self.in_position)
        self.inPositionButton.place(relx=0.05,rely=0.4,relwidth=0.9,relheight=.1)
        
        # Create label indicating drone distance from desired position.
        self.distanceFromDesiredLabel = tk.Label(self.frame, text=self.p_str.format(self.id, 0,0), bg='white', font=('Helvetica', 6), wraplength=100)
        self.distanceFromDesiredLabel.place(relx=0.05,rely=0.2,relwidth=0.9,relheight=.15)

        # Create labels for drones lat long and alt.
        self.latLabel = tk.Label(self.frame, text='',bg='white')
        self.latLabel.place(relx=0.05, rely=0.7, relwidth=0.9, relheight=0.1)
        self.longLabel = tk.Label(self.frame, text='',bg='white')
        self.longLabel.place(relx=0.05, rely=0.8, relwidth=0.9, relheight=0.1)
        self.altLabel = tk.Label(self.frame, text='',bg='white')
        self.altLabel.place(relx=0.05, rely=0.9, relwidth=0.9, relheight=0.1)
        
        return None

    def position_check(self):
        '''
            Called periodically while drone is in state = CONNECTED STATE.
            Calculates the distance from the drones position to the drones desired
            position.
            Updates the label with this info using self.p_str.
        '''

        # Get current and desired positions.
        current_pos = self.controller.drone_GPSs[self.id]
        desired_pos = self.controller.target_coords
        
        # Calculate distance from current to desired positions.
        distance_x = current_pos.x_distance(desired_pos)
        distance_y = current_pos.y_distance(desired_pos)

        # Update label with distances.
        self.distanceFromDesiredLabel.config(text=
            self.p_str.format(
            self.id,
            distance_y,
            distance_x))

        return None

    def in_position(self):
        '''
            Called when the in position button is pressed.
            Updates the drone state held by the controller to state = OFFBOARD_WAIT_STATE.
        '''

        # Move drone to state = OFFBOARD_WAIT_STATE
        self.controller.drone_states[self.id] = 2

        return None

    def offboard_wait(self):
        '''
            Called when controller receivees update indicating drone is in 
            state = OFFBOARD_WAIT.
            Creates label and button prompting for the drone to be moved to its start position.
        '''

        # Create label to indicate drones should be in offboard.
        self.distanceFromDesiredLabel.destroy()
        self.offboardPrompt = tk.Label(self.frame, text='Real drones should be moved to offboard state now.', bg='white', font=('Helvetica', 6), wraplength=100)
        self.offboardPrompt.place(relx=0.05,rely=0.2,relwidth=0.9,relheight=.15)

        # Creates button for moving drone into position.
        self.inPositionButton.destroy()
        self.moveToPosButton = tk.Button(self.frame, text='Move to position', command=self.move_to_pos)
        self.moveToPosButton.place(relx=0.05,rely=0.4,relwidth=0.9,relheight=.1)

        return None

    def move_to_pos(self):
        ''' 
            Called when the move to position button is pressed.
            Updates the drone state held by the controller to state = OFFBOARD_STATE.    
        '''

        # Move drone to state = OFFBOARD
        self.controller.drone_states[self.id] = 3
        
        # Change previous button and labels to reflect change
        self.moveToPosButton.destroy()
        self.offboardPrompt.destroy()

        return None
    
    def display_gps(self):
        '''
            Called periodically to update label indicating the drone's GPS pos.
        '''
        current_pos = self.controller.drone_GPSs[self.id]
        self.latLabel.config(text='lat: {:.4f}'.format(current_pos.lat))
        self.longLabel.config(text='long: {:.4f}'.format(current_pos.long))
        self.altLabel.config(text='alt: {:.4f}'.format(current_pos.alt))

        return None

class TX_GUI(DRONE_GUI):
    '''
        Child class of DRONE_GUI.
        Inherits all the same attributes and parameters.
        Additional methods included for Tx to send Rx drones the update indicating
        they are ready to start tracking the object.
    '''

    def track_prompt(self):
        '''
            Called when the TX drone is in state = OFFBOARD.
            Creates button prompting user to begin tracking of target.
        '''

        # Create tracking button
        self.trackButton = tk.Button(self.frame, text='TRACK', command=self.track)
        self.trackButton.place(relx=0.05,rely=0.4,relwidth=0.9,relheight=.1)

        return None

    def track(self):
        '''
            Called when tracking button is pressed.
            Updates the TX drone state held by the controller to state = READY.
        '''

        # Move TX drone to state = READY
        self.controller.drone_states[self.id] = 4
        self.trackButton.config(text='TRACKING', state=DISABLED)

        return None

def main():
    context = zmq.Context()
    controller = Controller(context)

    # -----------
    # Tkinter GUI
    # -----------
    root = tk.Tk()
    gui = CONTROLLERGUI(root,controller)
    root.protocol("WM_DELETE_WINDOW", gui.stop)
    root.mainloop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("TX: Interrupted.")
    except Exception as e:
        rospy.logfatal(traceback.format_exc())
    finally:
        sys.exit(0)

# drone_states = [0,0,0,0,0]
# states_strs = ["ON", "CONNECTED", "OFFBOARD WAIT", "OFFBOARD", "TRACKING", "HOLD", "FAIL"]
# drone_strs = ["TX", "RX1", "RX2", "RX3", "RX4"]
