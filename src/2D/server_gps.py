"""
filename: server_gps.py
author: Reka Norman
date: 16th October 2020
description: Reads output from GPS module and converts into Decimal Degrees output.
    GPS co-ordinates are then sent to another device running client_gps/mavros_offboard_postctl/rx using ZeroMQ
"""

import zmq
import serial
import struct
from gps import GPSCoord
import time

# Must match the one in mavros_offboard_posctl.py and rx.py or client_gps
PORT = 5556

# Linux location of USB port
SERIAL_PORT = '/dev/ttyUSB0'
# MacOS but may need to be redefined
# SERIAL_PORT = '../../../../../../../dev/tty.usbserial'

# GPS output begins with this
GPGGA = '$GPGGA'
# For date
WEEKDAYS = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
MONTHS = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']

# Offset transmitted position
X_OFFSET_M = 0
Y_OFFSET_M = 10
log_filename = "target_positions.txt"


def init_socket():
    """ Initialise ZeroMQ socket to send GPS co-ordinates """
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:{}".format(PORT))
    return socket


def main():
    """ Read output from GPS module and offset co-ordinates to client """
    # Socket to send readings to the client.
    socket = init_socket()

    # Open file
    log_file = open(log_filename, 'a')
    log_file.write("\n")

    # Set BAUD rate
    ser = serial.Serial(SERIAL_PORT, 4800, timeout=5)
    line = ser.readline()  # Read remaining junk line so that next line is clean

    while True:
        # Decode line
        line = ser.readline().decode('utf-8')
        splitline = line.split(",")

        # Module outputs lots of info, only use GPS
        if splitline[0] == GPGGA:

            latitude_nmea = splitline[2] + splitline[3]
            longitude_nmea = splitline[4] + splitline[5]
            try:
                # Convert to decimal degrees
                coord = GPSCoord.from_nmea(latitude_nmea, longitude_nmea)
            except ValueError:
                # Output from GPS occasionally erroneous
                print("Invalid GPS line:", line)
                continue

            # Add X and Y offsets
            coord = coord.add_x_offset(X_OFFSET_M).add_y_offset(Y_OFFSET_M)

            # Process date
            date = time.localtime(time.time())
            gps_time = "{}:{}:{}".format(splitline[1][:2], splitline[1][2:4], splitline[1][4:-4])
            time_sample = "{} {}  {} {} {}".format(WEEKDAYS[date.tm_wday], MONTHS[date.tm_mon - 1],
                                                   date.tm_mday, gps_time, date.tm_year)

            # Transmit to client program
            message = struct.pack("dd", coord.lat, coord.long)
            print("Sending: {}".format(coord))
            socket.send(message)
            log_file.write("{}, {}, {}\n".format(time_sample, coord.lat, coord.long))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted. Exiting...")
