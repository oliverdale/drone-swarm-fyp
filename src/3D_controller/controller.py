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
from tkinter.constants import RIDGE
import threading as thread
import time
from gps import GPSCoord

class Controller:

    def __init__(self, context):
        """
        
        """
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
        
        """
        success = True
        '''problem: cant get the packets using .connect - using .bind
            conflicts with tx.py'''
        # try:
        #     # PULL socket for receiving updates from the Rxs.
        #     self.receiver_rx_soc = self.context.socket(zmq.PULL)
        #     self.receiver_rx_soc.bind("tcp://*:{}".format(self.fp['COMMON']['RECEIVE_PORT']))
        #     # self.receiver_rx_soc.setsockopt(zmq.LINGER, 0)
        # except Exception as e:
        #     print("TX: Could not setup rx reciever connection")
        #     success = False

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

        # if self.receiver_rx_soc in active_sockets:
        #     message = self.receiver_tx_soc.recv(flags=zmq.NOBLOCK)
        #     update = RxUpdate.from_bytes(message)
        #     self.drone_GPSs[update.rx_id] = update.rx_coords
        #     self.drone_states[update.rx_id] = update.state
        #     print('rx ==============================')
        
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

    def test(self):
        print("YELLO")

    def start(self):

        self.controller_thread.start()
        self.startButton.config(state='disabled')

        for uav_gui in self.uav_guis.items():
            uav_gui[1].connecting()
        self.tx_guis['navi0'].connecting()
        self.targetWaitingLabel = tk.Label(self.canvas, text='Waiting for target server...', bg='#E4E4E4')
        self.targetWaitingLabel.place(relx=0,rely=0.3,relwidth=0.2,relheight=.1)

        self.run()

    def run(self):
        if self.controller.target_server_alive:
            self.targetWaitingLabel.destroy()
        self.check_states()            
        self.master.after(1000, self.run)

    def check_states(self):
        for state in self.controller.received_drone_states.items():
            rx_id = state[0]
            rx_state = state[1]


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
                

            if rx_state == 2:
                # Tx drones
                if rx_id == 0:
                    if self.tx_state_flag != 2:
                        self.tx_guis['navi{}'.format(rx_id)].offboard()
                        self.controller.drone_states[rx_id] = 2
                        self.tx_state_flag = 2
                # Rx drones
                elif rx_id >= 1:
                    if self.rx_state_flags['navi{}'.format(rx_id)] != 2:
                        self.uav_guis['navi{}'.format(rx_id)].offboard()
                        self.controller.drone_states[rx_id] = 2
                        self.rx_state_flags['navi{}'.format(rx_id)] = 2
                    

            if rx_state == 3:
                if rx_id == 0:
                    if self.tx_state_flag != 3:
                        self.tx_guis['navi{}'.format(rx_id)].track_check()
                        self.controller.drone_states[rx_id] = 3
                        self.tx_state_flag = 3

    def stop(self):
        self.controller_thread_stop = True
        self.controller.close_sockets
        self.master.destroy()

class DRONE_GUI():
    def __init__(self, master, id, controller):
        self.master = master
        self.id = id
        self.controller = controller
        self.frame = tk.Frame(master, bg='white', borderwidth=2, relief=RIDGE)
        self.frame.place(relx=0.35+0.15*(id-1),rely=0.05,relwidth=0.15,relheight=0.8)
        self.p_str = "RX{} is out of position. Move it {:.1f}m North and {:.1f}m East."

    def connecting(self):
        self.connectingLabel = tk.Label(self.frame, text="connecting...", fg="grey")
        self.connectingLabel.place(relx=0.05,rely=0.05,relwidth=0.9,relheight=.1)

    def connected(self):
        self.connectingLabel.config(text='Connected', bg='green', fg='white')
        self.connectedLabel = tk.Label(self.frame, text = 'In position?', bg='white')
        self.connectedLabel.place(relx=0.05,rely=0.4,relwidth=0.5,relheight=.1)
        self.connectedButton = tk.Button(self.frame, command=self.in_position)
        self.connectedButton.place(relx=0.6,rely=0.4,relwidth=0.2,relheight=.1)
        self.positionLabel = tk.Label(self.frame, text=self.p_str.format(self.id, 0,0), bg='white', font=('Helvetica', 6), wraplength=100)
        self.positionLabel.place(relx=0.05,rely=0.2,relwidth=0.9,relheight=.15)
        
    def position_check(self):
        current_pos = self.controller.drone_GPSs[self.id]
        # print(current_pos)
        desired_pos = self.controller.target_coords
        new_str = self.p_str.format(
            self.id,
            current_pos.y_distance(desired_pos),
            current_pos.x_distance(desired_pos)
        )
        self.positionLabel.config(text=new_str)

    def in_position(self):
        self.controller.drone_states[self.id] = 2

    def offboard(self):
        self.offboardLabel = tk.Label(self.frame, text='Move to start pos?')
        self.offboardLabel.place(relx=0.05,rely=0.4,relwidth=0.9,relheight=.15)
        self.offboardButton = tk.Button(self.frame, command=self.move_to_pos)
        self.offboardButton.place(relx=0.05,rely=0.55,relwidth=0.9,relheight=.15)

    def move_to_pos(self):
        self.controller.drone_states[self.id] = 3

class TX_GUI():
    def __init__(self, master, id, controller):
        self.master = master
        self.id = id
        self.controller = controller
        self.frame = tk.Frame(master, bg='white', borderwidth=2, relief=RIDGE)
        self.frame.place(relx=0.2,rely=0.05,relwidth=0.15,relheight=0.8)
        self.p_str = "RX{} is out of position. Move it {:.1f}m North and {:.1f}m East."

    def connecting(self):
        self.connectingLabel = tk.Label(self.frame, text="connecting...", fg="grey")
        self.connectingLabel.place(relx=0.05,rely=0.05,relwidth=0.9,relheight=.1)

    def connected(self):
        self.connectingLabel.config(text='Connected', bg='green', fg='white')
        self.connectedLabel = tk.Label(self.frame, text = 'In position?', bg='white')
        self.connectedLabel.place(relx=0.05,rely=0.4,relwidth=0.5,relheight=.1)
        self.connectedButton = tk.Button(self.frame, command=self.in_position)
        self.connectedButton.place(relx=0.6,rely=0.4,relwidth=0.2,relheight=.1)
        self.positionLabel = tk.Label(self.frame, text=self.p_str.format(self.id, 0,0), bg='white', font=('Helvetica', 6), wraplength=100)
        self.positionLabel.place(relx=0.05,rely=0.2,relwidth=0.9,relheight=.15)

    def position_check(self):
        current_pos = self.controller.drone_GPSs[self.id]
        desired_pos = self.controller.target_coords
        new_str = self.p_str.format(
            self.id,
            current_pos.y_distance(desired_pos),
            current_pos.x_distance(desired_pos)
        )
        self.positionLabel.config(text=new_str)

    def in_position(self):
        self.controller.drone_states[self.id] = 2

    def offboard(self):
        self.offboardLabel = tk.Label(self.frame, text='Move to start pos?')
        self.offboardLabel.place(relx=0.05,rely=0.4,relwidth=0.9,relheight=.15)
        self.offboardButton = tk.Button(self.frame, command=self.move_to_pos)
        self.offboardButton.place(relx=0.05,rely=0.55,relwidth=0.9,relheight=.15)

    def move_to_pos(self):
        self.controller.drone_states[self.id] = 3

    def track_check(self):
        self.trackButton = tk.Button(self.frame, text='TRACK', command=self.track)
        self.trackButton.place(relx=0.05,rely=0.7,relwidth=0.9,relheight=.15)
    def track(self):
        self.controller.drone_states[self.id] = 4

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
