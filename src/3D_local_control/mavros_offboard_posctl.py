#!/usr/bin/env python2
"""
filename: mavros_offboard_posctl.py
author: Zeb Barry
date: 16th October 2020
description: Program for controlling UAV using MAVROS to set GPS setpoints. Setpoints are received from server_gps.py
    and then sent to Pixhawk flight controller. Uses mavros_common.py to read MAVROS info from nodes.
"""

# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

import rospy
from common import get_parameters
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Quaternion, PoseStamped
from mavros_common import MultiMavrosCommon
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
from gps import GPSCoord


# True to convert GPS co-ordinates to local setpoints.
# Not needed, added during testing to check if global setpoints were working
LOCAL_SETPOINTS = True
SETPOINT_TOPIC = 'mavros/setpoint_position/' + ('local' if LOCAL_SETPOINTS else 'global')
POSITION_TYPE = PoseStamped if LOCAL_SETPOINTS else GeoPoseStamped




class MultiMavrosOffboardPosctl(MultiMavrosCommon):
    """ Controls a drone in Gazebo by sending position setpoints via MAVROS. """

    def set_up(self, uav_id, field):
        """ Setup MAVROS communication for UAV and start background thread to send setpoints.
        uav_id: ID number of UAV, -1 for only one UAV in sim otherwise int >= 0.
        """
        self.is_field = field
        flight_parameters = get_parameters()
        # If more than one UAV namespace must start with uav{ID}/
        if field:
            self.namespace = ''
        else:
            self.namespace = 'uav' + str(uav_id) + '/'

        rospy.init_node('offboard', anonymous=True)
        super(MultiMavrosOffboardPosctl, self).set_up(uav_id)

        


        self.default_altitude = flight_parameters[str(uav_id)]["DEF_ALTITUDE"]

                
        # Setup GPS setpoint publishing to MAVROS node
        self.ID = uav_id
        self.pos = POSITION_TYPE()
        self.pos_setpoint_pub = rospy.Publisher(
            self.namespace + SETPOINT_TOPIC, POSITION_TYPE, queue_size=1)

        # Send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def wait_for_offboard(self, timeout=None):
        """ Hangs until offboard mode is activated or timeout.
        timeout: Time in seconds
        """
        rate = rospy.Rate(1)
        time = 0
        while self.state.mode != "OFFBOARD":
            if self.is_shutdown():
                raise RuntimeError
            rospy.loginfo("Waiting for offboard mode | {0} seconds of {1}".
                          format(time, timeout))
            time += 1
            rate.sleep()
            if timeout is not None and time >= timeout:
                rospy.loginfo("Timeout while waiting for offboard mode | {0} seconds of {1}".
                              format(time, timeout))
                raise RuntimeError
        return True

    def wait_for_armed(self, timeout=None):
        
        """ Hangs until UAV armed or timeout.
        timeout: Time in seconds.
        """
        rate = rospy.Rate(1)
        time = 0
        while not self.state.armed:
            if self.is_shutdown():
                raise RuntimeError
            rospy.loginfo("Waiting for armed | {0} seconds of {1} | Armed = {2}".
                          format(time, timeout, self.state.armed))
            time += 1
            rate.sleep()
            if timeout is not None and time >= timeout:
                rospy.loginfo("Timeout while waiting for armed | {0} seconds of {1} | Armed = {2}".
                              format(time, timeout, self.state.armed))
                raise RuntimeError
        return True


    def take_off(self):
        """ Take off the UAV to the default altitude.
        If auto_arm is True, sets the mode to OFFBOARD and arms the UAV
        automatically. This should only be used for simulation testing, not in a
        practical flight test!
        Otherwise, waits for the mode to be set and UAV to be armed manually.
        """
        self.wait_for_topics(10)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        rospy.loginfo("UAV {0} taking off".format(self.ID))

        # If this is not a field test (i.e. a simulation) set the drone to offboard and take the drone off
        if not self.is_field:
            self.set_mode("OFFBOARD", 5)
            self.set_arm(True, 5)

        self.wait_for_offboard(None)
        self.wait_for_armed(None)

        self.initial_amsl = self.altitude.amsl
        print("Initial AMSL: ", self.initial_amsl)
        self.default_altitude += self.initial_amsl # This must be placed after the set up to ensure the reading is not zero.

        self.reach_position(self.global_position.latitude, self.global_position.longitude, self.default_altitude)
    
    def setup_practical(self):
        """ Waits for MAVROS topics to be correctly setup and then broadcasts initial position to MAVROS node.
        """
        rospy.loginfo("UAV {0} setting up".format(self.ID))
        self.wait_for_topics(10)
        self.initial_amsl = self.altitude.amsl
        self.default_altitude += self.initial_amsl # This must be placed after the set up to ensure the reading is not zero.
        self.reach_position(self.global_position.latitude, self.global_position.longitude, self.default_altitude)

    def land(self):
        """ Land UAV and disarm. """
        rospy.loginfo("UAV {0} landing".format(self.ID))
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

    def send_pos(self):
        """ Runs in a separate thread, publishing current position at a rate of 10 Hz. """
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            # Time stamp and publish setpoint
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def convert_global_to_local(self, lat, lon, alt):
        """ Convert global coords to local reference frame.
        lat, lon, alt: Latitude, Longitude and Altitude decimal degrees format
        """
        current_coords = GPSCoord(self.global_position.latitude, self.global_position.longitude, self.global_position.altitude)
        setpoint_coords = GPSCoord(lat, lon, alt)
        # Calculate difference in local co-ords based on global co-ords
        x_diff = current_coords.x_distance(setpoint_coords)
        y_diff = current_coords.y_distance(setpoint_coords)
        # Calculate new local setpoint based on distance to new position
        x = self.local_position.pose.position.x + x_diff
        y = self.local_position.pose.position.y + y_diff
        z = alt
        return x, y, z

    def set_position_setpoint(self, lat, lon, alt):
        """ Sets the position setpoint to the given lat, lon and alt.
        Method depends on whether global or local setpoints are being used.
        lat, lon, alt: Latitude, Longitude and Altitude decimal degrees format
        """
        if LOCAL_SETPOINTS:
            x, y, z = self.convert_global_to_local(lat, lon, alt)
            self.pos.pose.position.x = x
            self.pos.pose.position.y = y
            self.pos.pose.position.z = z
            rospy.logwarn('UAV{}: {}, {}, {}'.format(self.ID, x,y,z))
        else:
            self.pos.pose.position.latitude = lat
            self.pos.pose.position.longitude = lon
            self.pos.pose.position.altitude = alt
            self.pos.pose.

    def reach_position(self, lat, lon, alt=None):
        """ Set the position setpoint to the given values.
        lat, lon, alt: Latitude, Longitude and Altitude decimal degrees format
        """
        if alt == None:
            alt = self.default_altitude
        else:
            alt += self.initial_amsl
        self.set_position_setpoint(lat, lon, alt)
        # rospy.loginfo(
        #     "UAV {:.1f} attempting to reach position x: {:.6f}, y: {:.6f}, z: {:.6f} | current position x: {:.6f}, y: {:.6f}, z: {:.6f}".
        #         format(self.ID, lat, lon, alt,
        #                self.global_position.latitude,
        #                self.global_position.longitude,
        #                self.global_position.altitude))

        # Remove tf dependency for now to allow running with Python3
        # (this is equivalent to setting heading to North)
        self.pos.pose.orientation = Quaternion(0, 0, 0, 1)

    def get_current_position(self):
        """ Return the current position of the drone in the simulation. """
        return GPSCoord(self.global_position.latitude, self.global_position.longitude, self.global_position.altitude)

    def is_shutdown(self):
        """ Check if rospy is shut down """
        return rospy.is_shutdown()

