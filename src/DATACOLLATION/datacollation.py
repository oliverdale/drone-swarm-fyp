"""
filename: datacollate.py
author: Connor O'Reilly
date: 11-07-2021
description: function set that logs GPS coordinates
"""

import zmq
# import serial
import struct
import datetime
import time
import logging
from gps import GPSCoord

# Match mavros offboard
PORT = 5556

# MacOS uses directory
SERIAL_PORT = '../../../../../../../dev/tty.usbserial'

# GPS begins with
GPGGA = '$GPGGA'

# Makes output look nice
WEEKDAY = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
MONTHS = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']

# Log data
LOG_FORMAT_DES = "%(asctime)s :  GPS_DES = %(message)s"
LOG_FORMAT_TRU = "%(asctime)s :  GPS_TRUE = %(message)s"

log_file_test_DES_name = "test.txt"


# sets up logger -> desired coords
def setup_logger_des(log_name, log_file_dotLog, level=logging.DEBUG):
    logHandler = logging.FileHandler(log_file_dotLog)
    logHandler.setFormatter(LOG_FORMAT_DES)
    logger = logging.getLogger(log_name)
    logger.setLevel(level)
    logger.addHandler(logHandler)
    return logger


# sets up logger -> current coords
def setup_logger_true(log_name, log_file_dotLog, level=logging.DEBUG):
    logHandler = logging.FileHandler(log_file_dotLog)
    logHandler.setFormatter(LOG_FORMAT_TRU)
    logger = logging.getLogger(log_name)
    logger.setLevel(level)
    logger.addHandler(logHandler)
    return logger


# collate the data then send it
# Five drones, receive data and log the current coordinates ( can find this instance line 187 of rx.py )
# GPS coordinate log depend on the type of GPS coordinate; therefore, two different functions with similar functionality

# No init required as socket is initialised within other software

# log files GPS desired coords
logger_drone0_GPSDES = setup_logger_des('drone0_log_des', "GPS_DES_DRONE0.log")
log_file_test_DES = open(log_file_test_DES_name)
# log_file_test_DES.write("\n")
logger_drone1_GPSDES = setup_logger_des('drone1_log_des', "GPS_DES_DRONE1.log")
logger_drone2_GPSDES = setup_logger_des('drone2_log_des', "GPS_DES_DRONE2.log")
logger_drone3_GPSDES = setup_logger_des('drone3_log_des', "GPS_DES_DRONE3.log")
logger_drone4_GPSDES = setup_logger_des('drone4_log_des', "GPS_DES_DRONE4.log")

# files GPS true coords
logger_drone0_GPSTRUE = setup_logger_true('drone0_log_true', "GPS_TRUE_DRONE0.log")
logger_drone1_GPSTRUE = setup_logger_true('drone1_log_true', "GPS_TRUE_DRONE1.log")
logger_drone2_GPSTRUE = setup_logger_true('drone2_log_true', "GPS_TRUE_DRONE2.log")
logger_drone3_GPSTRUE = setup_logger_true('drone3_log_true', "GPS_TRUE_DRONE3.log")
logger_drone4_GPSTRUE = setup_logger_true('drone4_log_true', "GPS_TRUE_DRONE4.log")


# pass in swarming state

# sets up globals within class
class LogTimes:
    def __init__(self, time1_val, time2_val, time2_val_save, ticker):
        self.time1_val = time1_val
        self.time2_val = time2_val
        self.time2_val_save = time2_val_save
        self.ticker = ticker


def initLoggers():
    # log files GPS desired coords
    logger_drone0_GPSDES = setup_logger_des('drone0_log_des', "GPS_DES_DRONE0.log")
    # log_file_test_DES = open(log_file_test_DES_name)
    # log_file_test_DES.write("\n")
    logger_drone1_GPSDES = setup_logger_des('drone1_log_des', "GPS_DES_DRONE1.log")
    logger_drone2_GPSDES = setup_logger_des('drone2_log_des', "GPS_DES_DRONE2.log")
    logger_drone3_GPSDES = setup_logger_des('drone3_log_des', "GPS_DES_DRONE3.log")
    logger_drone4_GPSDES = setup_logger_des('drone4_log_des', "GPS_DES_DRONE4.log")
    # files GPS true coords
    logger_drone0_GPSTRUE = setup_logger_true('drone0_log_true', "GPS_TRUE_DRONE0.log")
    logger_drone1_GPSTRUE = setup_logger_true('drone1_log_true', "GPS_TRUE_DRONE1.log")
    logger_drone2_GPSTRUE = setup_logger_true('drone2_log_true', "GPS_TRUE_DRONE2.log")
    logger_drone3_GPSTRUE = setup_logger_true('drone3_log_true', "GPS_TRUE_DRONE3.log")
    logger_drone4_GPSTRUE = setup_logger_true('drone4_log_true', "GPS_TRUE_DRONE4.log")


time_GPSdesired = LogTimes(0.0, 0.0, 0.0, 0.0)
time_GPScurrent = LogTimes(0.0, 0.0, 0.0, 0.0)


# logs desired coordinates
# within update_logic function of swarming_logic.py
def logCOORD_desired(coords_des, drone_numa):
    # global time1_val_des, time2_val_des, time2_val_des_save, ticker_des now set at class
    time_GPSdesired.time1_val = time.time()
    if (time_GPSdesired.time1_val - time_GPSdesired.time2_val >= 0.25) \
            or (time_GPSdesired.time1_val - time_GPSdesired.time2_val_save >= 0.25):
        time_GPSdesired.ticker = 0
        coords_des_split = coords_des.split(",")
        if coords_des_split[0] == GPGGA:
            lat_nmea_coords_des_split = coords_des_split[2] + coords_des_split[3]
            long_nmea_coords_des_split = coords_des_split[4] + coords_des_split[5]
            coords_des_MSG = GPSCoord.from_nmea(lat_nmea_coords_des_split, long_nmea_coords_des_split)
            coords_str_time = "{}: ".format(datetime.datetime.today())
            coords_str_gps_lat_des = "{}, ".format(coords_des_MSG.lat)
            coords_str_gps_long_des = "{}\n".format(coords_des_MSG.long)
            drone_log_association_desired(coords_str_time, coords_str_gps_lat_des, coords_str_gps_long_des, drone_numa)
    else:
        # save first time stamp saved
        if time_GPSdesired.ticker <= 1:
            time_GPSdesired.time2_val_save = time.time()
    time_GPSdesired.ticker += 1
    time_GPSdesired.time2_val = time.time()
    # log_file_test_DES.write(coords_str_time + coords_str_gpsLatDes + coords_str_gpsLongDes)


# associate drone log file and drone number -> LOGS DESIRED COORDINATES
def drone_log_association_desired(coords_str_time, coords_str_gps_lat_des, coords_str_gps_long_des, drone_numa):
    if drone_numa == 0:
        logger_drone0_GPSDES.info(coords_str_time + coords_str_gps_lat_des + coords_str_gps_long_des)
        # log_file_test_DES.write(coords_str_time + coords_str_gpsLatDes + coords_str_gpsLongDes)
    elif drone_numa == 1:
        logger_drone1_GPSDES.info(coords_str_time + coords_str_gps_lat_des + coords_str_gps_long_des)
    elif drone_numa == 2:
        logger_drone2_GPSDES.info(coords_str_time + coords_str_gps_lat_des + coords_str_gps_long_des)
    elif drone_numa == 3:
        logger_drone3_GPSDES.info(coords_str_time + coords_str_gps_lat_des + coords_str_gps_long_des)
    elif drone_numa == 4:
        logger_drone4_GPSDES.info(coords_str_time + coords_str_gps_lat_des + coords_str_gps_long_des)

    # logs current coordinates


# use this function with lines 109 in tx.py and rx.py -> logs current coordinates
def logCOORD_current(coords_true, drone_numa):
    time_GPScurrent.time1_val = time.time()
    if ((time_GPScurrent.time1_val - time_GPScurrent.time2_val >= 0.25) or (
            time_GPScurrent.time1_val - time_GPScurrent.time2_val_save >= 0.25)):
        time_GPScurrent.ticker = 0
        coords_current_split = coords_true.split(",")
        if coords_current_split[0] == GPGGA:
            lat_nmea_coords_current_split = coords_current_split[2] + coords_current_split[3]
            long_nmea_coords_current_split = coords_current_split[4] + coords_current_split[5]
            coords_true_MSG = GPSCoord.from_nmea(lat_nmea_coords_current_split, long_nmea_coords_current_split)
            coords_str_time = "{}: ".format(datetime.datetime.today())
            coords_str_gps_lat_current = "{}, ".format(coords_true_MSG.lat)
            coords_str_gps_long_current = "{}\n".format(coords_true_MSG.long)
            drone_log_association_current(coords_str_time, coords_str_gps_lat_current, coords_str_gps_long_current,
                                          drone_numa)  # function below
    else:
        # save first time stamp saved
        if time_GPScurrent.ticker <= 1:
            time_GPScurrent.time2_val_save = time_GPScurrent.time2_val
    time_GPScurrent.ticker += 1
    time_GPScurrent.time2_val = time.time()


# associate drone log file and drone number -> LOGS CURRENT COORDINATES
def drone_log_association_current(coords_str_time, coords_str_gps_lat_current, coords_str_gps_long_current, drone_numa):
    if drone_numa == 0:
        logger_drone0_GPSTRUE.info(
            coords_str_time + coords_str_gps_lat_current + coords_str_gps_long_current)
    elif drone_numa == 1:
        logger_drone1_GPSTRUE.info(
            coords_str_time + coords_str_gps_lat_current + coords_str_gps_long_current)
    elif drone_numa == 2:
        logger_drone2_GPSTRUE.info(
            coords_str_time + coords_str_gps_lat_current + coords_str_gps_long_current)
    elif drone_numa == 3:
        logger_drone3_GPSTRUE.info(
            coords_str_time + coords_str_gps_lat_current + coords_str_gps_long_current)
    elif drone_numa == 4:
        logger_drone4_GPSTRUE.info(
            coords_str_time + coords_str_gps_lat_current + coords_str_gps_long_current)
