# External Libraries
import sys
import rospy
import traceback
import time

# Our stuff
from mavros_offboard_posctl import MultiMavrosOffboardPosctl



def main():

    # When running a simulation or practical flight test, perform UAV setup
    # before everything else so that the UAV is ready to fly.
    mavros_controller = MultiMavrosOffboardPosctl()
    mavros_controller.set_up(1, True)
    mavros_controller.setup_practical()

    t = time.time()
    # ==============================================================================
    # Main Loop 
    # ==============================================================================

    while True:
        try:
            if abs(time.time() - t) >= 1:
                print('======================================================')
                print('ASML: ', mavros_controller.altitude.amsl)
                print('Relative: ', mavros_controller.altitude.relative)
                print('Local: ', mavros_controller.altitude.local)
                print('Global: ', mavros_controller.global_position.altitude)
                print('Pose Alt: ', mavros_controller.pos.pose.position.altitude)
                #print('Pose z: ', mavros_controller.pos.pose.position.z)
                t = time.time()
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    main()
 
