"""
filename: packets.py
author: Reka Norman
date: 16th October 2020
description:
    Classes representing the network packets sent by the Rx and Tx UAVs.
"""

import struct

from gps import GPSCoord


class RxUpdate:
    def __init__(self, rx_id, state, timestamp, seq_no, rx_coords, range_reading, target_coords):
        self.rx_id = rx_id
        self.timestamp = timestamp
        self.seq_no = seq_no
        self.rx_coords = rx_coords
        self.range = range_reading
        self.state = state
        # The actual target position is sent for plotting at the Tx.
        self.target_coords = target_coords

    def __str__(self):
        return "ID {}, time {}, reading #{}, Rx coords {}, range {}, state {}".format(
            self.rx_id, self.timestamp, self.seq_no, self.rx_coords,
            self.range, self.state)
    
    @staticmethod
    def from_bytes(b):
        #print("packets.py from_bytes")
        var = [-1, -1, -1]
        target_var = [-1, -1, -1]
        (rx_id, state, timestamp, seq_no, lat, lon, alt, var[0], var[1], var[2], range_reading, target_lat,
         target_long, target_alt, target_var[0], target_var[1], target_var[2]) = struct.unpack("!iidi13d", b)
        return RxUpdate(rx_id, state, timestamp, seq_no, GPSCoord(lat, lon, alt, var),
                        range_reading, GPSCoord(target_lat, target_long, target_alt, target_var))

    def to_bytes(self):
        #print("packets.py to_bytes")
        b = struct.pack("!iidi3d", self.rx_id, self.state, self.timestamp, self.seq_no, self.rx_coords.lat, self.rx_coords.long, self.rx_coords.alt)
        for val in self.rx_coords.var:
            b += struct.pack("!d", val)
        b += struct.pack("!4d", self.range, self.target_coords.lat, self.target_coords.long, self.target_coords.alt)
        for val in self.target_coords.var:
            b += struct.pack("!d", val)
        return b


class TxUpdate:
    def __init__(self, states, target_coords, tx_GPS):
        self.target_coords = target_coords
        self.states = states # A list of current states for all the rxs
        # Only used for calculating emulated ranges, won't be needed when real
        # harmonic radar is used.
        self.tx_GPS = tx_GPS

    def __str__(self):
        return "Target position {}, Tx coords {}, state {}".format(
            self.target_coords, self.tx_GPS, self.states)

    @staticmethod
    def from_bytes(b):
        tx_var = [-1, -1, -1]
        target_var = [-1, -1, -1]
        states = [0]*5

        states[0], states[1], states[2], states[3], states[4], \
        target_lat, target_long, target_alt, \
        target_var[0], target_var[1], target_var[2], \
        tx_lat, tx_long, tx_alt, \
        tx_var[0], tx_var[1], tx_var[2] \
             = struct.unpack("!5i12d", b)

        return TxUpdate(states,
            GPSCoord(target_lat, target_long, target_alt, target_var),
            GPSCoord(tx_lat, tx_long, tx_alt, tx_var))

    def to_bytes(self):
        b = struct.pack("!5i", self.states[0], self.states[1], self.states[2], self.states[3], self.states[4])
        b += struct.pack("!3d", self.target_coords.lat, self.target_coords.long, self.target_coords.alt)
        for val in self.target_coords.var: b += struct.pack("!d", val)
        b += struct.pack("!3d", self.tx_GPS.lat, self.tx_GPS.long, self.tx_GPS.alt)
        for val in self.tx_GPS.var: b += struct.pack("!d", val)
        return b


class ContUpdate:
    def __init__(self, update_num, commands=[0]*5):
        self.update_num = update_num # The number of the update being sent to sync the communication between drones
        self.commands = commands # A list of numbers representing commands for each drone
  
    def __str__(self):
        return "Update Num {}, TX:{}, RX1:{}, RX2:{}, RX3:{}, RX4:{}".format(
            self.update_num, self.commands[0], self.commands[1], self.commands[2], self.commands[3], self.commands[4])

    @staticmethod
    def from_bytes(b):
        commands = [0]*5
        update_num, \
        commands[0], commands[1], commands[2], commands[3], commands[4], \
             = struct.unpack("!6i", b)

        return ContUpdate(update_num, commands)
   
    def to_bytes(self):
        b = struct.pack("!6i", self.update_num, self.commands[0], self.commands[1], self.commands[2], self.commands[3], self.commands[4])
        return b

class rxStateUpdate:
    def __init__(self, rxid, state, rx_coords=GPSCoord(0,0,0), target_coords=GPSCoord(0,0,0)):
        self.state = state
        self.rxid = rxid
        self.rx_coords = rx_coords
        self.target_coords = target_coords

    def __str__(self):
        return "RX{} => {} => {} => {}".format(self.rxid, self.state, self.rx_coords, self.target_coords)
            
    @staticmethod
    def from_bytes(b):
        rxid, state, lat,long,alt, target_lat,target_long,target_alt = struct.unpack("!2i6d", b)
        return rxStateUpdate(rxid, state, GPSCoord(lat,long,alt), GPSCoord(target_lat,target_long,target_alt))

    def to_bytes(self):
        b = struct.pack("!ii", self.rxid, self.state)
        b += struct.pack("!3d",self.rx_coords.lat, self.rx_coords.long, self.rx_coords.alt)
        b += struct.pack("!3d",self.target_coords.lat, self.target_coords.long, self.target_coords.alt)
        return b