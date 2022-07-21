# Tkinter GUI

import tkinter as tk
import time
import threading
from tkintertable.Tables import TableCanvas
from tkintertable.TableModels import TableModel

def button1_clicked():
    print("HELLO")

root = tk.Tk()

canvas = tk.Canvas(root, height=700, width=800, bg='white')
canvas.pack()

import zmq
import time

from zmq.sugar.constants import THREAD_PRIORITY_DFLT
from update_store import UpdateStore

from common import *
from zmq.sugar.context import T
from gps import GPSCoord
from packets import *
from controller import *

PARAMETERS_FILE = "flight_parameters_local.json"
UPDATE_PERIOD = 2
CONNECTING_STATE = 0
CONNECTED_STATE = 1

states = [0,0,0,0,0]
rx_coords = [0,0,0,0]
tx_coords = 0
state_names = ["ON_STATE", "CONNECTED_STATE", "OFFBOARD_WAIT_STATE",
                "OFFBOARD_STATE", "READY_STATE", "TRACKING_STATE", "HOLD_STATE", "FAIL_STATE"]
state_prompts = ["Confirm drone position.", "Move into formation", ]

class Gui:
    def __init__(self, root, controller):
        print("GUI intialisation")

        self.controller = controller

        # Control frame
        frame_control = tk.Frame(root, bg="#deeafc")
        frame_control.place(relx=0.05, rely=0.05, relwidth=0.25, relheight=0.9)

        # Controller buttons
        self.btn_state_ON = tk.Label(frame_control,text="ON STATE",bg="#ffffff")
        self.btn_state_ON.place(relx=0.1,rely=0.05,relheight=0.1,relwidth=0.8)

        # Controller buttons
        self.btn_state_CONNECTED = tk.Button(frame_control,text="ON STATE",bg="#ffffff", state='disabled', command=self.controller.send_update)
        self.btn_state_CONNECTED.place(relx=0.1,rely=0.2,relheight=0.1,relwidth=0.8)

        # # State control label
        # state_label = tk.Label(frame_control, text = "Confirm drone position.", bg="#80c1ff")
        # state_label.place(relx=0.0, rely=0.05, relwidth=0.25, relheight=0.2)
        # # State control button
        # self.state_button = tk.Button(frame_control, text="states", bg='white', fg='black',command=self.update_states)
        # self.state_button.place(relx = 0.0, rely=0.25, relwidth=0.25, relheight=0.3)


        # # Rx frame and labels
        # frame_rx = tk.Frame(root, bg='#80c1ff')
        # frame_rx.place(relx=0.3, rely=0.4, relwidth=0.65, relheight=0.2)

        # self.label_rx1 = tk.Label(frame_rx, text="RX1: state = {}".format(states[1]), fg='yellow')
        # self.label_rx1.place(relx=0.04,rely=0.1, relwidth=0.2, relheight=0.8)

        # self.label_rx2 = tk.Label(frame_rx, text="RX2: state = {}".format(states[2]), fg='yellow')
        # self.label_rx2.place(relx=0.28,rely=0.1, relwidth=0.2, relheight=0.8)

        # self.label_rx3 = tk.Label(frame_rx, text="RX3: state = {}".format(states[3]), fg='yellow')
        # self.label_rx3.place(relx=0controller
        # frame_tx = tk.Frame(root, bg="#80c1ff")
        # frame_tx.place(relx=0.05, rely=0.4, relwidth=0.2, relheight=0.2)

        # self.label_tx = tk.Label(frame_tx, text="TX", fg='yellow')
        # self.label_tx.place(relx=0.1,rely=0.1,relwidth=0.8,relheight=0.8)
        
        # # Table frame and entries
        # frame_tbl = tk.Frame(root, bg='grey')
        # frame_tbl.place(relx=0, rely=0.7, relwidth=1, relheight=0.3)
        # # Table view
        # table = TableCanvas(frame_tbl)
        # table.show() 
            
    def update_labels(self):
        """ Update labels with new state info"""
        global tx_coords 
        text_tx = "TX pos: {}".format(str(tx_coords))
        self.label_tx.config(text=text_tx)

        global rx_coords
        text_rx = "Rx pos: {}"
        self.label_rx1.config(text=text_rx.format(rx_coords[0]))
        self.label_rx2.config(text=text_rx.format(rx_coords[1]))
        self.label_rx3.config(text=text_rx.format(rx_coords[2]))
        self.label_rx4.config(text=text_rx.format(rx_coords[3]))

    def button_checker(self):
        """ Checks drone states to enable/disable certain buttons"""
        if (self.controller.state_num == 1):
            self.btn_state_ON.config(bg="#3cff00")
            self.btn_state_CONNECTED.config(state='normal')

    def run(self):
        while True:
            self.button_checker()
            # self.update_labels()
            time.sleep(2)

    def update_states(self):
        state = self.controller.update_states()
        self.state_button.config(text=state_names[state])

def start_controller(controller):
    t1 = threading.Thread(target=controller.run)
    t1.start()

def start_gui(gui):
    t2 = threading.Thread(target=gui.run)
    t2.start()

def main():
    context = zmq.Context()
    controller = Controller(context)
    gui = Gui(canvas, controller)

    # start_controller(controller)
    start_gui(gui)
    root.mainloop()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Controller Interrupted.")
    except Exception as e:
        rospy.logfatal(traceback.format_exc())
    finally:
        sys.exit(0)