"""
filename: packets.py
author: Reka Norman
date: 16th October 2020
description:
    Classes representing the network packets sent by the Rx and Tx UAVs.
"""

import struct

from gpsv2 import GPSCoord


class RxUpdate:
    def __init__(self, rx_id, timestamp, seq_no, rx_coords, range_reading,
                 target_coords):
        self.rx_id = rx_id
        self.timestamp = timestamp
        self.seq_no = seq_no
        self.rx_coords = rx_coords
        self.range = range_reading
        # The actual target position is sent for plotting at the Tx.
        self.target_coords = target_coords

    def __str__(self):
        return "ID {}, time {}, reading #{}, Rx coords {}, range {}".format(
            self.rx_id, self.timestamp, self.seq_no, self.rx_coords,
            self.range)

    @staticmethod
    def from_bytes(b):
        (rx_id, timestamp, seq_no, lat, lon, range_reading, target_lat,
         target_long) = struct.unpack("!ididdddd", b)
        return RxUpdate(rx_id, timestamp, seq_no, GPSCoord(lat, lon),
                        range_reading, GPSCoord(target_lat, target_long))

    def to_bytes(self):
        return struct.pack("!ididdddd", self.rx_id, self.timestamp,
                           self.seq_no, self.rx_coords.lat,
                           self.rx_coords.long, self.range,
                           self.target_coords.lat, self.target_coords.long)


class TxUpdate:
    def __init__(self, target_coords, tx_coords):
        self.target_coords = target_coords
        # Only used for calculating emulated ranges, won't be needed when real
        # harmonic radar is used.
        self.tx_coords = tx_coords

    def __str__(self):
        return "Target position {}, Tx coords {}".format(
            self.target_coords, self.tx_coords)

    @staticmethod
    def from_bytes(b):
        target_lat, target_long, tx_lat, tx_long = struct.unpack("!dddd", b)
        return TxUpdate(GPSCoord(target_lat, target_long),
                        GPSCoord(tx_lat, tx_long))

    def to_bytes(self):
        return struct.pack("!dddd", self.target_coords.lat,
                           self.target_coords.long, self.tx_coords.lat,
                           self.tx_coords.long)
