<<<<<<< Updated upstream
""" Test file for using scp to get file from Nics laptop."""

from paramiko import SSHClient
from paramiko.client import AutoAddPolicy

ssh = SSHClient()
ssh.set_missing_host_key_policy(AutoAddPolicy())
ssh.connect(
    hostname = '192.168.43.136',
    username = 'nick',
    password = 'nick42R',
    port = 22)
sftp_client=ssh.open_sftp()

sftp_client.get('/home/nick/hi.txt', 'downloaded.txt')

sftp_client.close()
ssh.close()
=======
# scp.py testing script
# Nicholas RAnum 20/07/2021

from paramiko import SSHClient
from scp import SCPClient

def main():
    # Create connection with navi1
    ssh = SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(SSHClient.AutoAddPolicy())

    print("hello")
    ssh.connect(hostname='92.168.43.86',username='ucnavi1',
                ,password='SERCUAV1',port=22)
    print("bye")
    scp = SCPClient(ssh.get_transport())

    scp.put('~/drone-swarm-fyp/src/3D/test.txt')

main()
>>>>>>> Stashed changes
