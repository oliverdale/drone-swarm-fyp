#!/usr/bin/env python

from common import *
from controller import *
import tkinter as tk
from tkinter.constants import DISABLED, LEFT, N, NO, RIDGE
import subprocess
import threading
import sys
from paramiko import SSHClient
from paramiko import AutoAddPolicy

# Get the required flight parameters. 
fp = get_parameters()

# SSH connection states
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
        self.ssh.set_missing_host_key_policy(AutoAddPolicy())
        try: 
            # Attempt to connect to the host. 
            self.ssh.connect(self.host, self.port, self.username, self.password, timeout=5)
        except Exception as e:
            print("Failed to connect to {}".format(self.username))

    def ssh_close(self):
        """
        Closes the SSH connection.
        """
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
    def __init__(self, master):
        self.master = master

        self.canvas = tk.Canvas(master, height=500, width=900, bg="#E6E6E6")
        self.canvas.pack()

        # Define GUIs for each receiver UAV. 
        self.navi1 = UAVGUI(master, 1)
        self.navi2 = UAVGUI(master, 2)
        self.navi3 = UAVGUI(master, 3)
        self.navi4 = UAVGUI(master, 4)

        self.drone_ids = [self.navi1.id, self.navi2.id, self.navi3.id, self.navi4.id]

        self.title = tk.Label(self.canvas, text="UAV Flight Controller", fg="black", font=('Helvetica', 12, 'bold'))
        self.title.place(relx=0, rely=0, relheight=0.05, relwidth=1)

        #------------------------------------------------------------
        # Simulated Drones GUI Design
        #------------------------------------------------------------
        self.frame = tk.Frame(master, bg="#E6E6E6", borderwidth=2, relief=RIDGE)
        self.frame.place(relx=(0.2*5)-0.2, rely=0.05, relwidth=0.2, relheight=1)

        self.NameLabel = tk.Label(self.frame, text="Simulated Drones", bg="#E6E6E6")
        self.NameLabel.place(relx=0.5, rely=0, relheight=0.05, relwidth=0.8, anchor='n')

        self.droneStatus = tk.Frame(self.frame, bg="#E6E6E6")
        self.droneStatus.place(relx=0.0, rely=0.05, relwidth=1, relheight=0.2)

        # Form a table showing the launch status of each UAV.
        self.configure_table()

        # Executes the simulation with the desired amount of simulated drones. 
        self.Execute1Button = tk.Button(self.frame, text="Execute launch",fg="black", bg = "#E6E6E6", command=lambda:self.launch_button("SIM"))
        self.Execute1Button.place(relx=0.3, rely=0.25, relheight=0.05, relwidth=0.6, anchor='n')

        # self.Execute1LogButton = tk.Button(self.frame, text="Check",fg="black", bg = "#E6E6E6", command=lambda:self.open_window("Launch"))
        # self.Execute1LogButton.place(relx=0.8, rely=0.25, relheight=0.05, relwidth=0.4, anchor='n')

        # self.Execute2LogButton = tk.Button(self.frame, text="Check",fg="black", bg = "#E6E6E6", command=lambda:self.open_window("Rx"))
        # self.Execute2LogButton.place(relx=0.8, rely=0.30, relheight=0.05, relwidth=0.4, anchor='n')

        # Executes the run_drones script with the desired amount of drones. 
        self.Execute2Button = tk.Button(self.frame, text="Execute rx.py",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.launch_button("RX"))
        self.Execute2Button.place(relx=0.3, rely=0.30, relheight=0.05, relwidth=0.6, anchor='n')
        
        # Check the drones selected for the simulation.
        self.update_simulations()

    def configure_table(self):
        """
        This configures a table showing the status for each drone. 
        A drone classified as 'Ready' will be launched in the simulation 
        and a 'Real' drone will be launched via SSH.
        """
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
        """
        This will update a list of drone ids that have been
        selected to be simulated. Updates every second. 
        """
        self.sim_ids = [] # Initialise simulation with the transmitter (id of 0).
        for navi in [self.navi1, self.navi2, self.navi3, self.navi4]:
            if not(navi.CheckFlight.get()):
                self.sim_ids.append(navi.id)

        # Update the status labels for each drone. 
        self.update_drone_status()

        # Update every second. 
        self.master.after(100, lambda:self.update_simulations()) 

    def update_drone_status(self):
        """
        This will update the status label for each drone within the table. 
        """
        drone_labels = [self.navi1Status, self.navi2Status, self.navi3Status, self.navi4Status]
        for id in self.drone_ids:
            if id in self.sim_ids:
                drone_labels[id - 1].config(text="Ready", fg="Green") 
            else:
                drone_labels[id - 1].config(text="Real", fg="Red") 

    def launch_button(self, type):
        """
        This will start a new thread that launches the simulation shell script or the 
        run_drones.py python script. Responds to both of the execute buttons.
        """
        id_string = "".join(str(id) for id in self.sim_ids) # Forms a string of the simulated drone ID's. 
        location = None
        if type == "SIM":
            cmd = ["./launch.sh", id_string] # Launch the simulation shell script.
            self.Execute2Button.config(state=tk.NORMAL)
            self.Execute1Button.config(state="disabled", text="Running")
            self.navi1.flightType.config(state=tk.DISABLED)
            self.navi2.flightType.config(state=tk.DISABLED)
            self.navi3.flightType.config(state=tk.DISABLED)
            self.navi4.flightType.config(state=tk.DISABLED)
        elif type =="RX":
            cmd = ["python3", "run_drones.py", "-sa", "-u", "0" + id_string]
            self.Execute2Button.config(state="disabled", text="Running")
        
        # Begins a new thread to avoid interrupting the tk mainloop. 
        exe = threading.Thread(target=self.execute_command, args=(cmd, location))
        exe.start()
   
    def execute_command(self, cmd, location):
        """
        Executes a given command within the OS. Connect the output to the PIPE to be buffered. 
        """
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, encoding='utf-8', stderr=subprocess.STDOUT, cwd=location)

        while True:
            # TODO: Buffer the stdout to view terminal output. This has been done for the SSH connections
            # but not for the local processes. This feature may not be needed.
            out = process.stdout.read(1)
            if out == '' and process.poll() != None:
                break
            if out != '':
                sys.stdout.write(out)
                sys.stdout.flush()
        
    def open_window(self, label):
        """
        This opens a new window to directly view the output of the commands run on the terminal. 
        Currently not used as the output has not been buffered. 
        """
        self.monitor = tk.Toplevel(self.master,  bg="#E6E6E6")
        self.monitor.geometry("450x300")
        self.monitor.title("Simulation" + label + " Terminal")
        #Create a Label in New window
        self.console = tk.Text(self.monitor, height=20)
        self.console.pack()

        self.stopCommandButton = tk.Button(self.monitor, text="Stop",fg="black", bg = "#E6E6E6")
        self.stopCommandButton.pack()
    
        #self.update_window()

class UAVGUI():
    def __init__(self, master, id=-1):
        self.master = master    
        self.id = id
        self.launch_buffer = ""
        self.rx_buffer = ""

        self.host = fp[str(id)]['IP_ADDRESS']
        self.port = 22
        self.username = fp[str(id)]['USERNAME']
        self.password = fp[str(id)]['PASSWORD']

        # Create a SSH connection attribute 
        self.connection = SSHUAV(self.host, self.port, self.username, self.password)
  
        #------------------------------------------------------------
        # NAVI GUI Design
        #------------------------------------------------------------
        self.CheckFlight = tk.IntVar()
    
        self.frame = tk.Frame(master, bg="#E6E6E6", borderwidth=2, relief=RIDGE)
        self.frame.place(relx=(0.2*self.id)-0.2, rely=0.05, relwidth=0.2, relheight=1)

        self.sshNameLabel = tk.Label(self.frame, text="UC Navi {}".format(id), bg="#E6E6E6")
        self.sshNameLabel.place(relx=0.5, rely=0, relheight=0.05, relwidth=0.8, anchor='n')

        self.flightType = tk.Checkbutton(self.frame, text = "Real drone", bg="#E6E6E6", variable = self.CheckFlight, onvalue = 1, offvalue = 0, command=lambda:self.update_connection())
        self.flightType.place(relx=0.5, rely=0.05, relheight=0.05, relwidth=0.8, anchor='n')

        self.sshConnectButton = tk.Button(self.frame, text="Connect via SSH",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.change_connection())
        self.sshConnectButton.place(relx=0.5, rely=0.1, relheight=0.05, relwidth=1, anchor='n')

        self.sshConnectLabel = tk.Label(self.frame, text="Simulated", bg="#E6E6E6", fg="blue")
        self.sshConnectLabel.place(relx=0.5, rely=0.15, relheight=0.05, relwidth=1, anchor='n')

        self.sshExecute1Button = tk.Button(self.frame, text="Execute launch",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.launch_drone())
        self.sshExecute1Button.place(relx=0.3, rely=0.2, relheight=0.05, relwidth=0.6, anchor='n')

        self.sshExecute1LogButton = tk.Button(self.frame, text="Check",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.open_window("LAUNCH"))
        self.sshExecute1LogButton.place(relx=0.8, rely=0.2, relheight=0.05, relwidth=0.4, anchor='n')

        self.sshExecute2LogButton = tk.Button(self.frame, text="Check",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.open_window("RX"))
        self.sshExecute2LogButton.place(relx=0.8, rely=0.25, relheight=0.05, relwidth=0.4, anchor='n')

        self.sshExecute2Button = tk.Button(self.frame, text="Execute rx.py",fg="black", bg = "#E6E6E6", state=tk.DISABLED, command=lambda:self.launch_rx())
        self.sshExecute2Button.place(relx=0.3, rely=0.25, relheight=0.05, relwidth=0.6, anchor='n')

        # Checks if the drone is to be simulated or not. If not, the status of the SSH connection is checked.
        self.update_connection()

    def update_connection(self):
        """
        Updates the connection labels within the GUI depending on the connection 
        state. The execution and log buttons are disabled unless there is an SSH
        connection.
        """
        if (self.CheckFlight.get() == 1):
            self.sshConnectButton.config(state="active")
            connection_status = self.connection.check_connection() # Check SSH connection.

            if connection_status == CONNECTED:
                self.sshConnectLabel.config(text="Connected", fg="Green") 
                self.sshConnectButton.config(text="Disconnect via SSH")
                self.sshExecute1Button.config(state=tk.NORMAL)
                self.sshExecute2Button.config(state=tk.NORMAL)
                self.sshExecute1LogButton.config(state=tk.NORMAL)
                self.sshExecute2LogButton.config(state=tk.NORMAL)
            else:
                self.sshConnectLabel.config(text="Disconnected", fg="Red") 
                self.sshConnectButton.config(text="Connect via SSH")
                self.sshExecute1Button.config(state=tk.DISABLED)
                self.sshExecute2Button.config(state=tk.DISABLED)
                self.sshExecute1LogButton.config(state=tk.DISABLED)
                self.sshExecute2LogButton.config(state=tk.DISABLED)
        else:
            self.sshConnectLabel.config(text="Simulated", fg="Blue") 
            self.sshConnectButton.config(state=tk.DISABLED)
            self.sshExecute1Button.config(state=tk.DISABLED)
            self.sshExecute2Button.config(state=tk.DISABLED)
            self.sshExecute1LogButton.config(state=tk.DISABLED)
            self.sshExecute2LogButton.config(state=tk.DISABLED)

        # Function loops at 2 Hz.
        self.master.after(500, lambda:self.update_connection()) 
    
    def open_window(self, command_type):
        """
        This opens a new window to directly view the output of the commands via SSH. 
        The window is updated with the corresponding buffer depending on the command 
        type.
        """
        self.monitor = tk.Toplevel(self.master,  bg="#E6E6E6")
        self.monitor.geometry("450x300")
        self.monitor.title("UAV {} ".format(self.id) + command_type + " Terminal")
        
        self.console = tk.Text(self.monitor, height=20)
        self.console.pack()

        self.stopCommandButton = tk.Button(self.monitor, text="Stop",fg="black", bg = "#E6E6E6")
        self.stopCommandButton.pack()
    
        self.update_window(command_type)

    def update_window(self, command_type):
        """
        Clears the current text within the window and adds the updated buffer
        showing the terminal output. 
        """
        if (self.console is not None) and self.console.winfo_exists():
            self.console.delete(1.0, tk.END)
            self.console.insert(tk.END, self.readOutput(command_type))
            self.console.see(tk.END)
            self.master.after(1000, lambda:self.update_window(command_type))

    def launch_drone(self):
        """
        Starts a new thread to execute the ros launch file on the real drone.
        """
        if (self.CheckFlight.get() == 1): 
            command = "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; cd drone-swarm-fyp;cd PX4-Autopilot; roslaunch launch/insect_tracking_mavros.launch"
            command_type = "LAUNCH"
            connection_status = self.connection.check_connection()
            if connection_status == CONNECTED:
                self.sshExecute1Button.config(state="disabled", text="Running")
                self.ssh_execute(command, command_type)
                exe = threading.Thread(target=self.ssh_execute, args=(command, ))
                exe.start()
            else:
                print("Not connected to {}".format(self.connection.username))

    def launch_rx(self):
        """
        Starts a new thread to execute the rx.py python script on the real drone. 
        """
        if (self.CheckFlight.get() == 1):
            command = "cd drone-swarm-fyp/src/3D_new_controller; rx.py -f {}".format(self.id) # Command for physical drone
            command_type = "RX"
            connection_status = self.connection.check_connection()
            if connection_status == CONNECTED:
                self.ssh_execute(command, command_type)
            else:
                print("Not connected to {}".format(self.connection.username))
 
    def ssh_execute(self, command, command_type):
        """
        Executes a command via SSH and reads the output into a buffer. 
        """
        try:
            stdin, stdout, stderr = self.connection.ssh.exec_command(command, get_pty=True)
            for line in iter(stdout.readline, ""):
                if command_type == "LAUNCH":
                    self.launch_buffer += line
                elif command_type == "RX":
                    self.rx_buffer += line
            return True
        except Exception as e:
            print("Connection lost : %s" %e)
            return False

    def readOutput(self, command_type):
        """
        Helper function to return the corresponding buffer to update the window.
        """
        if command_type == "LAUNCH":
            return self.launch_buffer
        elif command_type == "RX":
            return self.rx_buffer

    def change_connection(self):
        """
        This function allows the UAV to be connected or disconnected from.
        """
        if (self.CheckFlight.get() == 1):
            connection_status = self.connection.check_connection()
            if connection_status == CONNECTED:
                self.connection.ssh_close()
            else:
                self.connection.ssh_connect()

def main ():
    root = tk.Tk()
    main_ui = MainGUI(root)
    root.mainloop()

if __name__ == "__main__":
        main()
