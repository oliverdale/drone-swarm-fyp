#!/usr/bin/env python

#from common import *
from logging import root
from sys import stdout
import tkinter as tk
from tkinter import StringVar, Text
from tkinter.constants import DISABLED, LEFT, N, NO, RIDGE
import numpy as np

import subprocess
import os
import re
import threading
import queue
import sys

from paramiko import SSHClient
from paramiko import AutoAddPolicy

#flight_parameters = get_parameters()

DISCONNECTED = 0
CONNECTED = 1

class SSHUAV():
    def __init__(self, host, port, username, password):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.ssh = SSHClient()

    def ssh_connect(self):
        print("connection")
        self.ssh.set_missing_host_key_policy(AutoAddPolicy())
        self.ssh.connect(self.host, self.port, self.username, self.password, timeout=5)

    def ssh_close(self):
        self.ssh.close()

    def check_connection(self):
        """
        This will check if the connection is still availlable.
        Return (bool) : True if it's still alive, False otherwise.
        """
        try:
            self.ssh.exec_command('ls', timeout=5)
            return CONNECTED
        except Exception as e:
            return DISCONNECTED
    
class MainGUI:
    def __init__(self, master, navi1):
        self.master = master

        self.canvas = tk.Canvas(master, height=500, width=900, bg="#E6E6E6")
        self.canvas.pack()

        self.navi1 = UAVGUI(master, navi1, 1)
        self.navi2 = UAVGUI(master, navi1, 2)
        self.navi3 = UAVGUI(master, navi1, 3)
        self.navi4 = UAVGUI(master, navi1, 4)

        self.drone_ids = [self.navi1.id, self.navi2.id, self.navi3.id, self.navi4.id]

        self.xterm_id = False
        self.launch_buffer = []

        self.title = tk.Label(self.canvas, text="UAV Flight Controller", fg="black", font=('Helvetica', 12, 'bold'))
        self.title.place(relx=0, rely=0, relheight=0.05, relwidth=1)

        self.frame = tk.Frame(master, bg="#E6E6E6", borderwidth=2, relief=RIDGE)
        self.frame.place(relx=(0.2*5)-0.2, rely=0.05, relwidth=0.2, relheight=1)

        self.sshNameLabel = tk.Label(self.frame, text="Simulated Drones", bg="#E6E6E6")
        self.sshNameLabel.place(relx=0.5, rely=0, relheight=0.05, relwidth=0.8, anchor='n')

        self.droneStatus = tk.Frame(self.frame, bg="#E6E6E6")
        self.droneStatus.place(relx=0.0, rely=0.05, relwidth=1, relheight=0.2)

        self.configure_table()

        self.sshExecute1Button = tk.Button(self.frame, text="Execute launch",fg="black", bg = "#E6E6E6", command=lambda:self.launch_sim())
        self.sshExecute1Button.place(relx=0.3, rely=0.25, relheight=0.05, relwidth=0.6, anchor='n')

        self.sshExecute1LogButton = tk.Button(self.frame, text="Check",fg="black", bg = "#E6E6E6", command=lambda:self.open_window("Launch"))
        self.sshExecute1LogButton.place(relx=0.8, rely=0.25, relheight=0.05, relwidth=0.4, anchor='n')

        self.sshExecute2LogButton = tk.Button(self.frame, text="Check",fg="black", bg = "#E6E6E6", command=lambda:self.open_window("Rx"))
        self.sshExecute2LogButton.place(relx=0.8, rely=0.30, relheight=0.05, relwidth=0.4, anchor='n')

        self.sshExecute2Button = tk.Button(self.frame, text="Execute rx.py",fg="black", bg = "#E6E6E6", command=lambda:self.launch_rx())
        self.sshExecute2Button.place(relx=0.3, rely=0.30, relheight=0.05, relwidth=0.6, anchor='n')
        
        self.update_simulations()

    def configure_table(self):
        # configure the grid
        self.droneStatus.columnconfigure(0, weight=1)
        self.droneStatus.columnconfigure(1, weight=1)

        self.navi1Label = tk.Label(self.droneStatus, text="UC Navi 1", bg="#E6E6E6", fg="black", justify=LEFT)
        self.navi1Label.grid(row=0, column=0, sticky = tk.W)

        self.navi2Label = tk.Label(self.droneStatus, text="UC Navi 2", bg="#E6E6E6", fg="black")
        self.navi2Label.grid(row=1, column=0, sticky = tk.W)

        self.navi3Label = tk.Label(self.droneStatus, text="UC Navi 3", bg="#E6E6E6", fg="black")
        self.navi3Label.grid(row=2, column=0, sticky = tk.W)

        self.navi4Label = tk.Label(self.droneStatus, text="UC Navi 4", bg="#E6E6E6", fg="black")
        self.navi4Label.grid(row=3, column=0, sticky = tk.W)

        self.navi1Status = tk.Label(self.droneStatus, text="Ready", bg="#E6E6E6", fg="green")
        self.navi1Status.grid(row=0, column=1)

        self.navi2Status = tk.Label(self.droneStatus, text="Ready", bg="#E6E6E6", fg="green")
        self.navi2Status.grid(row=1, column=1)

        self.navi3Status = tk.Label(self.droneStatus, text="Ready", bg="#E6E6E6", fg="green")
        self.navi3Status.grid(row=2, column=1)

        self.navi4Status = tk.Label(self.droneStatus, text="Ready", bg="#E6E6E6", fg="green")
        self.navi4Status.grid(row=3, column=1)


    def update_simulations(self):   
        self.sim_ids = [] # Initialise simulation with the transmitter (id of 0).
        for navi in [self.navi1, self.navi2, self.navi3, self.navi4]:
            if not(navi.CheckFlight.get()):
                self.sim_ids.append(navi.id)

        self.update_drone_status()
        self.master.after(100, lambda:self.update_simulations()) 

    def update_drone_status(self):
        drone_labels = [self.navi1Status, self.navi2Status, self.navi3Status, self.navi4Status]
        for id in self.drone_ids:
            if id in self.sim_ids:
                drone_labels[id - 1].config(text="Ready", fg="Green") 
            else:
                drone_labels[id - 1].config(text="Real", fg="Red") 

    def launch_sim(self):
        #TO DO: Launch command through xterm. And then allow window to communicate to it. 
        id_string = "".join(str(id) for id in self.sim_ids)
        cmd = ["python3", "src/3D/test.py"]

        exe = threading.Thread(target=self.execute, args=(cmd, ))
        exe.start()
   


    def execute(self, cmd):
        print("here")


        # p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=1, text=True, shell=True)
        # while p.poll() is None:
        #     msg = p.stdout.readline().strip() # read a line from the process output
        #     if msg:
        #         print(msg)

        test = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=1)
        for line in iter(test.stdout.readline, b''):
            print(line)
    # def open_window(self, label):
    #     self.monitor = tk.Toplevel(self.master,  bg="#E6E6E6")
    #     self.monitor.geometry("450x300")
    #     self.monitor.title("Simulation " + label + " Terminal")
    #     self.xterm_id= self.monitor.winfo_id()
    #     subprocess.Popen(["xterm", "-into", str(self.xterm_id), "-geometry", "300x300"],
    #             stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    def open_window(self, label):
        self.monitor = tk.Toplevel(self.master,  bg="#E6E6E6")
        self.monitor.geometry("450x300")
        self.monitor.title("Simulation" + label + " Terminal")
        #Create a Label in New window
        self.console = tk.Text(self.monitor, height=20)
        self.console.pack()

        self.stopCommandButton = tk.Button(self.monitor, text="Stop",fg="black", bg = "#E6E6E6")
        self.stopCommandButton.pack()
    
        self.update_window()

    def update_window(self):
        if (self.console is not None) and self.console.winfo_exists():
            self.console.delete(1.0, tk.END)
            self.console.insert(tk.END, self.launch_buffer)
            self.master.after(1000, lambda:self.update_window())

        
 
class UAVGUI():
    def __init__(self, master, connection="local", id=-1):

        self.master = master    
        self.id = id
        self.buffer = ""
        self.connection = connection

        #------------------------------------------------------------
        # NAVI GUI Design
        #------------------------------------------------------------

        self.CheckFlight = tk.IntVar()
    
        self.frame = tk.Frame(master, bg="#E6E6E6", borderwidth=2, relief=RIDGE)
        self.frame.place(relx=(0.2*self.id)-0.2, rely=0.05, relwidth=0.2, relheight=1)

        self.sshNameLabel = tk.Label(self.frame, text="UC Navi {}".format(id), bg="#E6E6E6")
        self.sshNameLabel.place(relx=0.5, rely=0, relheight=0.05, relwidth=0.8, anchor='n')

        self.flightType = tk.Checkbutton(self.frame, text = "Real drone", bg="#E6E6E6", variable = self.CheckFlight, onvalue = 1, offvalue = 0, command=lambda:self.update_type(connection))
        self.flightType.place(relx=0.5, rely=0.05, relheight=0.05, relwidth=0.8, anchor='n')

        self.sshConnectButton = tk.Button(self.frame, text="Connect via SSH",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.change_connection())
        self.sshConnectButton.place(relx=0.5, rely=0.1, relheight=0.05, relwidth=1, anchor='n')

        self.sshConnectLabel = tk.Label(self.frame, text="Simulated", bg="#E6E6E6", fg="blue")
        self.sshConnectLabel.place(relx=0.5, rely=0.15, relheight=0.05, relwidth=1, anchor='n')

        self.sshExecute1Button = tk.Button(self.frame, text="Execute launch",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.launch_drone())
        self.sshExecute1Button.place(relx=0.3, rely=0.2, relheight=0.05, relwidth=0.6, anchor='n')

        self.sshExecute1LogButton = tk.Button(self.frame, text="Check",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.open_window("Launch"))
        self.sshExecute1LogButton.place(relx=0.8, rely=0.2, relheight=0.05, relwidth=0.4, anchor='n')

        self.sshExecute2LogButton = tk.Button(self.frame, text="Check",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.open_window("Rx"))
        self.sshExecute2LogButton.place(relx=0.8, rely=0.25, relheight=0.05, relwidth=0.4, anchor='n')

        self.sshExecute2Button = tk.Button(self.frame, text="Execute rx.py",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.launch_rx())
        self.sshExecute2Button.place(relx=0.3, rely=0.25, relheight=0.05, relwidth=0.6, anchor='n')

        self.update_connection()

    def update_connection(self):
        if (self.CheckFlight.get() == 1):
            connection_status = self.connection.check_connection()

            if connection_status == CONNECTED:
                self.sshConnectLabel.config(text="Connected", fg="Green") #Update label with next text.
                self.sshConnectButton.config(text="Disconnect via SSH")
                self.sshExecute1Button.config(state=tk.NORMAL)
                self.sshExecute2Button.config(state=tk.NORMAL)
                self.sshExecute1LogButton.config(state=tk.NORMAL)
                self.sshExecute2LogButton.config(state=tk.NORMAL)
            else:
                self.sshConnectLabel.config(text="Disconnected", fg="Red") #Update label with next text.
                self.sshConnectButton.config(text="Connect via SSH")
                self.sshExecute1Button.config(state=tk.DISABLED)
                self.sshExecute2Button.config(state=tk.DISABLED)
                self.sshExecute1LogButton.config(state=tk.DISABLED)
                self.sshExecute2LogButton.config(state=tk.DISABLED)

        self.master.after(500, lambda:self.update_connection()) #calls update_label function again after 0.5 second. (500 milliseconds.)
    
    def update_type(self, connection):
        if (self.CheckFlight.get() == 1):
            self.connection = SSHUAV(connection.host, connection.port, connection.username, connection.password)
            self.sshConnectButton.config(state="active")
        else:
            self.connection = "local"
            self.sshConnectLabel.config(text="Simulated", fg="Blue") #Update label with next text.
            self.sshConnectButton.config(state=tk.DISABLED)
            self.sshExecute1Button.config(state=tk.DISABLED)
            self.sshExecute2Button.config(state=tk.DISABLED)
            self.sshExecute1LogButton.config(state=tk.DISABLED)
            self.sshExecute2LogButton.config(state=tk.DISABLED)

    def open_window(self, label):
        self.monitor = tk.Toplevel(self.master,  bg="#E6E6E6")
        self.monitor.geometry("450x300")
        self.monitor.title("UAV {} ".format(self.id) + label + " Terminal")
        #Create a Label in New window
        self.console = tk.Text(self.monitor, height=20)
        self.console.pack()

        self.stopCommandButton = tk.Button(self.monitor, text="Stop",fg="black", bg = "#E6E6E6")
        self.stopCommandButton.pack()
    
        self.update_window()

    def update_window(self):
        if (self.console is not None) and self.console.winfo_exists():
            self.console.delete(1.0, tk.END)
            self.console.insert(tk.END, self.readOutput())
            self.master.after(1000, lambda:self.update_window())

    def launch_drone(self):
        if (self.CheckFlight.get() == 1): 
            command = "python3 test.py"
            connection_status = self.connection.check_connection()
            if connection_status == CONNECTED:
                self.sshExecute1Button.config(state="disabled", text="Running")
                #self.ssh_execute('source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; cd drone-swarm-fyp;cd PX4-Autopilot; source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default; roslaunch px4 full_swarm_simulated.launch')
                #self.ssh_execute('source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; cd drone-swarm-fyp;cd PX4-Autopilot; roslaunch launch/insect_tracking_mavros.launch')
                exe = threading.Thread(target=self.ssh_execute, args=(command, ))
                exe.start()
            else:
                print("Not connected to {}".format(self.connection.username))
        else:
            command = "python3 test.py"

    def launch_rx(self):
        if (self.CheckFlight.get() == 1):
            command = 'python3 test.py' # Command for physical drone
            connection_status = self.connection.check_connection()
            if connection_status == CONNECTED:
                self.ssh_execute(command)
            else:
                print("Not connected to {}".format(self.connection.username))
        else:
            command = 'python3 test.py' # Command for simulated environment


    def ssh_execute(self, command):
        try:
            stdin, stdout, stderr = self.connection.ssh.exec_command(command, get_pty=True)
            for line in iter(stdout.readline, ""):
                self.buffer += line
            print("Finished")
            self.sshExecute1Button.config(state="active", text="Execute launch")
            return True
        except Exception as e:
            print("Connection lost : %s" %e)
            return False

    def readOutput(self):
        return self.buffer

    def change_connection(self):
        if (self.CheckFlight.get() == 1):
            connection_status = self.connection.check_connection()
            if connection_status == CONNECTED:
                self.connection.ssh_close()
            else:
                print("frfrf")
                self.connection.ssh_connect()

class NAVIDetails():
    def __init__(self, host, port, username, password):
        self.host = host
        self.port = port
        self.username = username
        self.password = password

def main ():
    # Defining the SSHUAV class. 
    host = "192.168.20.60"
    port = 22
    username = "nic"
    password = "nick42R"

    navi1 = NAVIDetails(host, port, username, password)
    
    #------------------------------------------------------------
    # Tkinter GUI Design
    #------------------------------------------------------------
    root = tk.Tk()
    main_ui = MainGUI(root, navi1)
    root.mainloop()

if __name__ == "__main__":
        main()