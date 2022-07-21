"""
filename: client_gps.py
author: Zeb Barry
date: 16th October 2020
description: Receives GPS co-ordinates from device running server_gps.py and records to file.
    Two files are used, one for current setpoint and one to record all setpoints
"""
import zmq

# Will need to change this if communicating between different machines.
IP_ADDRESS = "localhost"

# Must match the one in server_gps.py
PORT = 5556


def main():
    # Socket receive readings from the server.
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://{}:{}".format(IP_ADDRESS, PORT))
    socket.setsockopt(zmq.SUBSCRIBE, b"")

    while True:
        message = socket.recv().decode('utf-8')
        time_sample, latitude, longitude = message.split(',')
        print("time_sample: {}, latitude: {}, longtitude: {}".format(
            time_sample, latitude, longitude))

        messagedata = time_sample + ',' + latitude + ',' + longitude
        current = open("data_gps.txt", "w")
        storage = open("data_storage.txt", "a")
        # Write current location to current file
        current.write(messagedata + '\n')
        # Append current location to storage file
        storage.write(messagedata + '\n')
        current.close()
        storage.close()


main()
