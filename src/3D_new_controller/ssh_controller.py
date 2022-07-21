from common import *

from paramiko import SSHClient
from paramiko import AutoAddPolicy

flight_parameters = get_parameters()


class SSHUAV:
    def __init__(self, host, port, username, password):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.ssh = SSHClient()

    def sshconnect(self):
        self.ssh.set_missing_host_key_policy(AutoAddPolicy())
        self.ssh.connect(self.host, self.port, self.username, self.password)
        
    def sshexecute(self, command):
        stdin, stdout, stderr = self.ssh.exec_command(command)
        lines = stdout.readlines()
        print(lines)


def main ():
    """
    Testing the SSHUAV class. 
    """
    host = "192.168.20.55"
    port = 22
    username = "nick"
    password = "nick42R"
    command = "cd Desktop; ls"

    connect = input("Press y to connect to nics computer.\n")

    while (connect != 'y'):
        connect = input("Press y to connect to nics computer.\n")

    nicscomputer = SSHUAV(host, port, username, password)
    nicscomputer.sshconnect()

    execute = input("Press y to execute a command on nics computer.\n")

    while (execute != 'y'):
        execute = input("Press y to execute a command on nics computer.\n")
    nicscomputer.sshexecute(command)

if __name__ == "__main__":
        main()