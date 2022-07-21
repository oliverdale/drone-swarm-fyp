"""
filename: datacollate.py
author: Connor O'Reilly
date: 18-07-2021
description: function set that logs GPS coordinates
prior to test and simulation days, use log_init_..._start (removes all contents in file)
logger. sync not an issue - improved readability. needs tidy up - works on last years code and tx.py and rx.py scripts within this directory
"""

from os import close

import zmq
import serial
import socket
import struct
import datetime
import time
import logging
import gps
import gps
from mavros_offboard_posctl import MultiMavrosOffboardPosctl

TX_ID = 0

# Makes output look nice
WEEKDAY = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
MONTHS = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']

log_file_drone0_DES_name = "log_data/drone0des.txt"
log_file_drone1_DES_name = "log_data/drone1des.txt"
log_file_drone2_DES_name = "log_data/drone2des.txt"
log_file_drone3_DES_name = "log_data/drone3des.txt"
log_file_drone4_DES_name = "log_data/drone4des.txt"

log_file_drone0_CUR_name = "log_data/drone0cur.txt"
log_file_drone1_CUR_name = "log_data/drone1cur.txt"
log_file_drone2_CUR_name = "log_data/drone2cur.txt"
log_file_drone3_CUR_name = "log_data/drone3cur.txt"
log_file_drone4_CUR_name = "log_data/drone4cur.txt"


# collate the data then send it
# Five drones, receive data and log the current coordinates ( can find this instance line 187 of rx.py )
# GPS coordinate log depend on the type of GPS coordinate; therefore, two different functions with similar functionality

# No init required as socket is initialised within other software

# pass in swarming state

# sets up globals within class


# run coords_store first, then log
def log_init_des(drone_numa):
    if drone_numa == 0:
        log_file_drone0_DES = open(log_file_drone0_DES_name, 'a')
        # log_file_drone0_DES.write("==============================================================================\n")
        log_file_drone0_DES.write("start\n")
        log_file_drone0_DES.close()
    elif drone_numa == 1:
        log_file_drone1_DES = open(log_file_drone1_DES_name, 'a')
        # log_file_drone1_DES.write("==============================================================================\n")
        log_file_drone1_DES.write("start\n")
        log_file_drone1_DES.close()
    elif drone_numa == 2:
        log_file_drone2_DES = open(log_file_drone2_DES_name, 'a')
        # log_file_drone2_DES.write("==============================================================================\n")
        log_file_drone2_DES.write("start\n")
        log_file_drone2_DES.close()
    elif drone_numa == 3:
        log_file_drone3_DES = open(log_file_drone3_DES_name, 'a')
        # log_file_drone3_DES.write("==============================================================================\n")
        log_file_drone3_DES.write("start\n")
        log_file_drone3_DES.close()
    elif drone_numa == 4:
        log_file_drone4_DES = open(log_file_drone4_DES_name, 'a')
        # log_file_drone4_DES.write("==============================================================================\n")
        log_file_drone4_DES.write("start\n")
        log_file_drone4_DES.close()


def log_init_cur(drone_numa):
    if drone_numa == 0:
        log_file_drone0_CUR = open(log_file_drone0_CUR_name, 'a')
        # log_file_drone0_CUR.write("==============================================================================\n")
        log_file_drone0_CUR.write("start\n")
        log_file_drone0_CUR.close()
    elif drone_numa == 1:
        log_file_drone1_CUR = open(log_file_drone1_CUR_name, 'a')
        # log_file_drone1_CUR.write("==============================================================================\n")
        log_file_drone1_CUR.write("start\n")
        log_file_drone1_CUR.close()
    elif drone_numa == 2:
        log_file_drone2_CUR = open(log_file_drone2_CUR_name, 'a')
        # log_file_drone2_CUR.write("==============================================================================\n")
        log_file_drone2_CUR.write("start\n")
        log_file_drone2_CUR.close()
    elif drone_numa == 3:
        log_file_drone3_CUR = open(log_file_drone3_CUR_name, 'a')
        # log_file_drone3_CUR.write("==============================================================================\n")
        log_file_drone3_CUR.write("start\n")
        log_file_drone3_CUR.close()
    elif drone_numa == 4:
        log_file_drone4_CUR = open(log_file_drone4_CUR_name, 'a')
        # log_file_drone4_CUR.write("==============================================================================\n")
        log_file_drone4_CUR.write("start\n")
        log_file_drone4_CUR.close()


def log_init_des_start(drone_numa):
    if drone_numa == 0:
        log_file_drone0_DES = open(log_file_drone0_DES_name, 'w')
        log_file_drone0_DES.truncate(0)
        log_file_drone0_DES.close()
    elif drone_numa == 1:
        log_file_drone1_DES = open(log_file_drone1_DES_name, 'w')
        log_file_drone1_DES.truncate(0)
        log_file_drone1_DES.close()
    elif drone_numa == 2:
        log_file_drone2_DES = open(log_file_drone2_DES_name, 'w')
        log_file_drone2_DES.truncate(0)
        log_file_drone2_DES.close()
    elif drone_numa == 3:
        log_file_drone3_DES = open(log_file_drone3_DES_name, 'w')
        log_file_drone3_DES.truncate(0)
        log_file_drone3_DES.close()
    elif drone_numa == 4:
        log_file_drone4_DES = open(log_file_drone4_DES_name, 'w')
        log_file_drone4_DES.truncate(0)
        log_file_drone4_DES.close()


def log_init_cur_start(drone_numa):
    if drone_numa == 0:
        log_file_drone0_CUR = open(log_file_drone0_CUR_name, 'w')
        log_file_drone0_CUR.truncate(0)
        log_file_drone0_CUR.close()
    elif drone_numa == 1:
        log_file_drone1_CUR = open(log_file_drone1_CUR_name, 'w')
        log_file_drone1_CUR.truncate(0)
        log_file_drone1_CUR.close()
    elif drone_numa == 2:
        log_file_drone2_CUR = open(log_file_drone2_CUR_name, 'w')
        log_file_drone2_CUR.truncate(0)
        log_file_drone2_CUR.close()
    elif drone_numa == 3:
        log_file_drone3_CUR = open(log_file_drone3_CUR_name, 'w')
        log_file_drone3_CUR.truncate(0)
        log_file_drone3_CUR.close()
    elif drone_numa == 4:
        log_file_drone4_CUR = open(log_file_drone4_CUR_name, 'w')
        log_file_drone4_CUR.truncate(0)
        log_file_drone4_CUR.close()


class LogGPS:
    def __init__(self, lat, lon, drone_numa, time_epoch):  # , lt_cur):  # , time1_val, time2_val, time2_val_save, ticker):
        self.time_epoch_des = time_epoch
        self.time_epoch_previous_des = time_epoch
        self.time_epoch_cur = time_epoch
        self.time_epoch_previous_cur = time_epoch
        # self.lt_cur = lt_cur
        self.drone_numa = drone_numa
        self.lat = lat
        self.lon = lon

    def time_check_des(self, drone_numa, lat, lon, t_rec):
        # timing within this function to enable synchronous logging of data and improve reusability
        self.time_epoch_des = time.time()
        time_string = str(self.time_epoch_des)
        time_string_10 = time_string[9]
        time_string_13 = time_string[12]
        time_string_previous = str(self.time_epoch_previous_des)
        time_string_10_previous = time_string_previous[9]
        if (drone_numa == TX_ID):
            if ((time_string_10 != time_string_10_previous) or (time_string_13 == (str(0))) or (time_string_13 == str(9))):
                self.logCOORD_des(drone_numa, lat, lon, t_rec)
        else:
            if ((time_string_13 == (str(0))) or (time_string_13 == str(9))):
                self.logCOORD_des(drone_numa, lat, lon, t_rec)
        self.time_epoch_previous_des = self.time_epoch_des

    def logCOORD_des(self, drone_numa, lat, lon, t_rec):
        lat_gps_s = lat
        long_gps_s = lon
        date = time.localtime(time.time())
        # gps_coords_time = "{}:{}:{}".format(gps_des_split[1][:2], gps_des_split[1][2:4], gps_des_split[1][4:-4])
        time_sample_des = "{} {}  {} {}".format(WEEKDAY[date.tm_wday], MONTHS[date.tm_mon - 1],
                                                date.tm_mday, date.tm_year)
        gps_coords_lat_des = "{}".format(lat_gps_s)
        gps_coords_long_des = "{}".format(long_gps_s)
        drone_log_ass_des(time_sample_des, gps_coords_lat_des, gps_coords_long_des, drone_numa, t_rec)

    def time_check_cur(self, drone_numa, real_lat, real_lon, t_rec_sig_cur):
        self.time_epoch_cur = time.time()
        time_string = str(self.time_epoch_cur)
        time_string_10 = time_string[9]
        time_string_13 = time_string[12]
        time_string_previous = str(self.time_epoch_previous_cur)
        time_string_10_previous = time_string_previous[9]
        if (drone_numa == TX_ID):
            if ((time_string_10 != time_string_10_previous) or ((time_string_13 == (str(0))) or (time_string_13 == str(9)))):
                self.logCOORD_cur(drone_numa, real_lat, real_lon, t_rec_sig_cur)
        else:
            if ((time_string_13 == (str(0))) or (time_string_13 == str(9))):
                self.logCOORD_cur(drone_numa, real_lat, real_lon, t_rec_sig_cur)
        self.time_epoch_previous_cur = self.time_epoch_cur

    def logCOORD_cur(self, drone_numa, real_lat, real_lon, t_rec_sig_cur):
        real_lat = str(real_lat)
        real_lat_gps_array = real_lat.split(", ")
        lat_gps_real = str(real_lat_gps_array[0])
        long_gps_real = str(real_lat_gps_array[1])
        lat_gps_real = lat_gps_real.replace('(', '')
        long_gps_real = long_gps_real.replace(')', '')
        date = time.localtime(time.time())
        # gps_coords_time = "{}:{}:{}".format(gps_des_split[1][:2], gps_des_split[1][2:4], gps_des_split[1][4:-4])
        time_sample_real = "{} {}  {} {}".format(WEEKDAY[date.tm_wday], MONTHS[date.tm_mon - 1],
                                                 date.tm_mday, date.tm_year)
        gps_coords_lat_real = "{}".format(lat_gps_real)
        gps_coords_long_real = "{}".format(long_gps_real)
        drone_log_ass_cur(time_sample_real, gps_coords_lat_real, gps_coords_long_real, drone_numa, t_rec_sig_cur)


# logs time of log (not time of data sample)
def drone_log_ass_des(time_sample_des, coords_str_gps_lat_des, coords_str_gps_long_des, drone_numa, t_rec):
    if drone_numa == 0:
        log_file_drone0_DES = open(log_file_drone0_DES_name, 'a')
        log_file_drone0_DES.write("\n{} {}: {}, {}".format(t_rec, time_sample_des, coords_str_gps_lat_des,
                                                           coords_str_gps_long_des))
        log_file_drone0_DES.close()
        print("=================================================logged")
    elif drone_numa == 1:
        log_file_drone1_DES = open(log_file_drone1_DES_name, 'a')
        log_file_drone1_DES.write("\n{} {}: {}, {}".format(t_rec, time_sample_des, coords_str_gps_lat_des,
                                                           coords_str_gps_long_des))
        log_file_drone1_DES.close()
    elif drone_numa == 2:
        log_file_drone2_DES = open(log_file_drone2_DES_name, 'a')
        log_file_drone2_DES.write("\n{} {}: {}, {}".format(t_rec, time_sample_des, coords_str_gps_lat_des,
                                                           coords_str_gps_long_des))
        log_file_drone2_DES.close()
    elif drone_numa == 3:
        log_file_drone3_DES = open(log_file_drone3_DES_name, 'a')
        log_file_drone3_DES.write("\n{} {}: {}, {}".format(t_rec, time_sample_des, coords_str_gps_lat_des,
                                                           coords_str_gps_long_des))
        log_file_drone3_DES.close()
    elif drone_numa == 4:
        log_file_drone4_DES = open(log_file_drone4_DES_name, 'a')
        log_file_drone4_DES.write("\n{} {}: {}, {}".format(t_rec, time_sample_des, coords_str_gps_lat_des,
                                                           coords_str_gps_long_des))
        log_file_drone4_DES.close()


def drone_log_ass_cur(time_sample_cur, coords_str_gps_lat_cur, coords_str_gps_long_cur, drone_numa, t_rec_sig_cur):
    if drone_numa == 0:
        log_file_drone0_CUR = open(log_file_drone0_CUR_name, 'a')
        log_file_drone0_CUR.write("\n{} {}: {}, {}".format(t_rec_sig_cur, time_sample_cur, coords_str_gps_lat_cur,
                                                           coords_str_gps_long_cur))
        log_file_drone0_CUR.close()
    elif drone_numa == 1:
        log_file_drone1_CUR = open(log_file_drone1_CUR_name, 'a')
        log_file_drone1_CUR.write("\n{} {}: {}, {}".format(t_rec_sig_cur, time_sample_cur, coords_str_gps_lat_cur,
                                                           coords_str_gps_long_cur))
        log_file_drone1_CUR.close()
    elif drone_numa == 2:
        log_file_drone2_CUR = open(log_file_drone2_CUR_name, 'a')
        log_file_drone2_CUR.write("\n{} {}: {}, {}".format(t_rec_sig_cur, time_sample_cur, coords_str_gps_lat_cur,
                                                           coords_str_gps_long_cur))
        log_file_drone2_CUR.close()
    elif drone_numa == 3:
        log_file_drone3_CUR = open(log_file_drone3_CUR_name, 'a')
        log_file_drone3_CUR.write("\n{} {}: {}, {}".format(t_rec_sig_cur, time_sample_cur, coords_str_gps_lat_cur,
                                                           coords_str_gps_long_cur))
        log_file_drone3_CUR.close()
    elif drone_numa == 4:
        log_file_drone4_CUR = open(log_file_drone4_CUR_name, 'a')
        log_file_drone4_CUR.write("\n{} {}: {}, {}".format(t_rec_sig_cur, time_sample_cur, coords_str_gps_lat_cur,
                                                           coords_str_gps_long_cur))
        log_file_drone4_CUR.close()
