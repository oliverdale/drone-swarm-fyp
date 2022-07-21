import string
import matplotlib.pyplot as plt
import time
import numpy as np

flight_time = "09-17-2021_15-23"

flight_file_name = "/flight_" + flight_time + "/"

log_file_drone0_name = "logs" + flight_file_name + "drone0_" + flight_time #+ ".txt"
log_file_drone1_name = "logs" + flight_file_name + "drone1_" + flight_time #+ ".txt"
log_file_drone2_name = "logs" + flight_file_name + "drone2_" + flight_time #+ ".txt"
log_file_drone3_name = "logs" + flight_file_name + "drone3_" + flight_time #+ ".txt"
log_file_drone4_name = "logs" + flight_file_name + "drone4_" + flight_time #+ ".txt"

data_lat_array_drone1_cur, data_lon_array_drone1_cur, data_height_array_drone1_cur = [], [], []
data_lat_array_drone1_cur_mark, data_lon_array_drone1_cur_mark = [], []
data_lat_array_drone1_des, data_lon_array_drone1_des, data_height_array_drone1_des = [], [], []
data_lat_array_drone1_des_mark, data_lon_array_drone1_des_mark = [], []

data_lat_array_drone2_cur, data_lon_array_drone2_cur, data_height_array_drone2_cur = [], [], []
data_lat_array_drone2_cur_mark, data_lon_array_drone2_cur_mark, data_height_array_drone2_cur_mark = [], [], []
data_lat_array_drone2_des, data_lon_array_drone2_des, data_height_array_drone2_des = [], [], []
data_lat_array_drone2_des_mark, data_lon_array_drone2_des_mark, data_height_array_drone2_des_mark = [], [], []

data_lat_array_drone3_cur, data_lon_array_drone3_cur, data_height_array_drone3_cur = [], [], []
data_lat_array_drone3_cur_mark, data_lon_array_drone3_cur_mark, data_height_array_drone3_cur_mark = [], [], []
data_lat_array_drone3_des, data_lon_array_drone3_des, data_height_array_drone3_des = [], [], []
data_lat_array_drone3_des_mark, data_lon_array_drone3_des_mark, data_height_array_drone3_des_mark = [], [], []


# drone states
ON_STATE = 0
CONECTED_STATE = 1
OFFBOARD_WAIT_STATE = 2
OFFBOARD_STATE = 3
READY_STATE = 4
TRACKING_STATE = 5
HOLD_STATE = 6
FAIL_STATE = 7




def main():
    index = 0
    data_time_previous = None

    data_all = log_file_drone0_name
    data_all_temp = open(log_file_drone0_name, "r")

    data_ff_1 = data_all_temp.readline()
    data_ff_1 = data_all_temp.readline()
    data_ff_1 = data_all_temp.readlines()

    #data_des = log_file_drone1_DES_name
    #data_temp_des = open(data_des, "r")

    #data_ff_des_1 = data_temp_des.readline()
    #data_ff_des_1 = data_temp_des.readlines()

    for line in data_ff_1:
        if (line == ' '):
            continue

        data_array = line.strip().split("; ")
        data_time = str(data_array[0])
        data_time = data_time.strip().split(":")
        if (str(data_time[0]) == "STOP"):
            print("stop")
            break
        data_time = str(data_time[3])
        print(data_array[1])
        data_states_list = data_array[1]
        
        data_states_list_swarm_state = data_states_list.strip().split("]")
        data_states_list_swarm_state = data_states_list_swarm_state[1] 
        if (data_states_list_swarm_state == (TRACKING_STATE or HOLD_STATE or FAIL_STATE)) and ((data_time[0] != data_time_previous) or (data_time_previous == None)):
            data_target_filt_cart_coords = data_array[2]
            data_target_filt_gps = data_array[4]
            data_coords_target = data_array[5] # important data
            data_filt_tx_cart_coords = data_array[6] ### data_year.replace(':', '')
            data_filt_gps = data_array[7]
            data_coords_real = data_array[8] # important data
            data_multilateration_error = data_array[9]
            data_mode = data_array[10]

            # target coords    
            data_coords_target = data_coords_target.strip().split(", ")
            data_coords_target_lat = data_coords_target[0]
            data_coords_target_lon = data_coords_target[1]
            data_coords_target_height = data_coords_target[2]
            data_coords_target_lat = data_coords_target_lat.strip().split("(")
            data_coords_target_lat = data_coords_target_lat[1]
            data_coords_target_lon = data_coords_target_lon.strip(" ")
            data_coords_target_height = data_coords_target_height.strip(")")

            # real coords
            data_coords_real = data_coords_real.strip().split(", ")
            data_coords_real_lat = data_coords_real[0]
            data_coords_real_lon = data_coords_real[1]
            data_coords_real_height = data_coords_real[2]
            data_coords_real_lat = data_coords_real_lat.strip().split("(")
            data_coords_real_lat = data_coords_real_lat[1]
            data_coords_real_lon = data_coords_real_lon.strip(" ")
            data_coords_real_height = data_coords_real_height.strip(")")


            # set negative float target coords
            if (data_coords_target_lat[0] == '-'):
                data_coords_target_lat = data_coords_target_lat.replace('-', '')
                data_coords_target_lat = float(data_coords_target_lat)
                data_coords_target_lat = -1 * data_coords_target_lat
            else:
                data_coords_target_lat = float(data_coords_target_lat)

            if (data_coords_target_lon[0] == '-'):
                data_coords_target_lon = data_coords_target_lon.replace('-', '')
                data_coords_target_lon = float(data_coords_target_lon)
                data_coords_target_lon = -1 * data_coords_target_lon
            else:
                data_coords_target_lon = float(data_coords_target_lon)

            print(data_coords_target_lat)
            print(data_coords_target_lon)
            
            # set negative float real coords
            if (data_coords_real_lat[0] == '-'):
                data_coords_real_lat = data_coords_real_lat.replace('-', '')
                data_coords_real_lat = float(data_coords_real_lat)
                data_coords_real_lat = -1 * data_coords_real_lat
            else:
                data_coords_real_lat = float(data_coords_real_lat)

            if (data_coords_real_lon[0] == '-'):
                data_coords_real_lon = data_coords_real_lon.replace('-', '')
                data_coords_real_lon = float(data_coords_real_lon)
                data_coords_real_lon = -1 * data_coords_real_lon
            else:
                data_coords_real_lon = float(data_coords_real_lon)

            print(data_coords_real_lat)
            print(data_coords_real_lon)

            # append to individual lists - target coords
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone1_des_mark.append(data_coords_target_lat)
                data_lon_array_drone1_des_mark.append(data_coords_target_lon)
            data_lat_array_drone1_des.append(data_coords_target_lat)
            data_lon_array_drone1_des.append(data_coords_target_lon)
            data_time_previous = data_time[0]

            #append to individual lists - real coords
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone1_cur_mark.append(data_coords_real_lat)
                data_lon_array_drone1_cur_mark.append(data_coords_real_lon)
            data_lat_array_drone1_cur.append(data_coords_real_lat)
            data_lon_array_drone1_cur.append(data_coords_real_lon)
            data_time_previous = data_time[0]
            index += 1


    data_time_previous = None
    index = 0

    data_all = log_file_drone1_name
    data_all_temp = open(log_file_drone1_name, "r")

    data_ff_2 = data_all_temp.readline()
    data_ff_2 = data_all_temp.readlines()

    # receivers have different layout to the transmitter
    for line in data_ff_2:

        if (line == ' ') or (line[0] == "T"):
            continue

        data_array = line.strip().split("; ")
        data_time = str(data_array[0])
        data_time = data_time.strip().split(":")
        if (str(data_time[0]) == "STOP"):
            print("stop")
            break
        data_time_second = str(data_time[3])

        data_receiver_state = data_array[1]
        data_mode = str(data_array[2])

        if (((data_time_second[0] != data_time_previous) or (data_time_previous == None))):
        #(data_mode == "OFFBOARD") and 
            data_gps_des = data_array[3]
            if (data_array[3] == "None"):
                continue
            data_gps_cur = data_array[4]
           
            # target coords    
            data_gps_des = data_gps_des.strip().split(", ")
            data_gps_des_lat = data_gps_des[0]
            data_gps_des_lon = data_gps_des[1]
            data_gps_des_height = data_gps_des[2]
            data_gps_des_lat = data_gps_des_lat.strip().split("(")
            data_gps_des_lat = data_gps_des_lat[1] # removes the GPS part of the string
            data_gps_des_lon = data_gps_des_lon.strip(" ")
            data_gps_des_height = float(data_gps_des_height.strip(")"))

            # real coords
            data_gps_cur = data_gps_cur.strip().split(", ")
            data_gps_cur_lat = data_gps_cur[0]
            data_gps_cur_lon = data_gps_cur[1]
            data_gps_cur_height = data_gps_cur[2]
            data_gps_cur_lat = data_gps_cur_lat.strip().split("(")
            data_gps_cur_lat = data_gps_cur_lat[1]
            data_gps_cur_lon = data_gps_cur_lon.strip(" ")
            data_gps_cur_height = float(data_gps_cur_height.strip(")"))


            # set negative float target coords
            if (data_gps_des_lat[0] == '-'):
                data_gps_des_lat = data_gps_des_lat.replace('-', '')
                data_gps_des_lat = float(data_gps_des_lat)
                data_gps_des_lat = -1 * data_gps_des_lat
            else:
                data_gps_des_lat = float(data_gps_des_lat)

            if (data_gps_des_lon[0] == '-'):
                data_gps_des_lon = data_gps_des_lon.replace('-', '')
                data_gps_des_lon = float(data_gps_des_lon)
                data_gps_des_lon = -1 * data_gps_des_lon
            else:
                data_gps_des_lon = float(data_gps_des_lon)

            print(data_gps_des_lat)
            print(data_gps_des_lon)
            
            # set negative float real coords
            if (data_gps_cur_lat[0] == '-'):
                data_gps_cur_lat = data_gps_cur_lat.replace('-', '')
                data_gps_cur_lat = float(data_gps_cur_lat)
                data_gps_cur_lat = -1 * data_gps_cur_lat
            else:
                data_gps_cur_lat = float(data_gps_cur_lat)

            if (data_gps_cur_lon[0] == '-'):
                data_gps_cur_lon = data_gps_cur_lon.replace('-', '')
                data_gps_cur_lon = float(data_gps_cur_lon)
                data_gps_cur_lon = -1 * data_gps_cur_lon
            else:
                data_gps_cur_lon = float(data_gps_cur_lon)

            print(data_gps_cur_lat)
            print(data_gps_cur_lon)
            print(data_gps_cur_height)

            # append to individual lists - target coords
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone2_des_mark.append(data_gps_des_lat)
                data_lon_array_drone2_des_mark.append(data_gps_des_lon)
                data_height_array_drone2_des_mark.append(data_gps_des_height)
            data_lat_array_drone2_des.append(data_gps_des_lat)
            data_lon_array_drone2_des.append(data_gps_des_lon)
            data_height_array_drone2_des.append(data_gps_des_height)

            #append to individual lists - real coords
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone2_cur_mark.append(data_gps_cur_lat)
                data_lon_array_drone2_cur_mark.append(data_gps_cur_lon)
                data_height_array_drone2_cur_mark.append(data_gps_cur_height)
            data_lat_array_drone2_cur.append(data_gps_cur_lat)
            data_lon_array_drone2_cur.append(data_gps_cur_lon)
            data_height_array_drone2_cur.append(data_gps_cur_height)
            data_time_previous = data_time_second[0]
            index += 1

    data_time_previous = None
    index = 0

    data_all = log_file_drone1_name
    data_all_temp = open(log_file_drone1_name, "r")

    data_ff_3 = data_all_temp.readline()
    data_ff_3 = data_all_temp.readlines()

    # receivers have different layout to the transmitter
    for line in data_ff_3:

        if (line == ' ') or (line[0] == "T"):
            continue

        data_array = line.strip().split("; ")
        data_time = str(data_array[0])
        data_time = data_time.strip().split(":")
        if (str(data_time[0]) == "STOP"):
            print("stop")
            break
        data_time_second = str(data_time[3])

        data_receiver_state = data_array[1]
        data_mode = str(data_array[2])

        if (((data_time_second[0] != data_time_previous) or (data_time_previous == None))):
        #(data_mode == "OFFBOARD") and 
            data_gps_des = data_array[3]
            if (data_array[3] == "None"):
                continue
            data_gps_cur = data_array[4]
           
            # target coords    
            data_gps_des = data_gps_des.strip().split(", ")
            data_gps_des_lat = data_gps_des[0]
            data_gps_des_lon = data_gps_des[1]
            data_gps_des_height = data_gps_des[2]
            data_gps_des_lat = data_gps_des_lat.strip().split("(")
            data_gps_des_lat = data_gps_des_lat[1] # removes the GPS part of the string
            data_gps_des_lon = data_gps_des_lon.strip(" ")
            data_gps_des_height = float(data_gps_des_height.strip(")"))

            # real coords
            data_gps_cur = data_gps_cur.strip().split(", ")
            data_gps_cur_lat = data_gps_cur[0]
            data_gps_cur_lon = data_gps_cur[1]
            data_gps_cur_height = data_gps_cur[2]
            data_gps_cur_lat = data_gps_cur_lat.strip().split("(")
            data_gps_cur_lat = data_gps_cur_lat[1]
            data_gps_cur_lon = data_gps_cur_lon.strip(" ")
            data_gps_cur_height = float(data_gps_cur_height.strip(")"))


            # set negative float target coords
            if (data_gps_des_lat[0] == '-'):
                data_gps_des_lat = data_gps_des_lat.replace('-', '')
                data_gps_des_lat = float(data_gps_des_lat)
                data_gps_des_lat = -1 * data_gps_des_lat
            else:
                data_gps_des_lat = float(data_gps_des_lat)

            if (data_gps_des_lon[0] == '-'):
                data_gps_des_lon = data_gps_des_lon.replace('-', '')
                data_gps_des_lon = float(data_gps_des_lon)
                data_gps_des_lon = -1 * data_gps_des_lon
            else:
                data_gps_des_lon = float(data_gps_des_lon)

            print(data_gps_des_lat)
            print(data_gps_des_lon)
            
            # set negative float real coords
            if (data_gps_cur_lat[0] == '-'):
                data_gps_cur_lat = data_gps_cur_lat.replace('-', '')
                data_gps_cur_lat = float(data_gps_cur_lat)
                data_gps_cur_lat = -1 * data_gps_cur_lat
            else:
                data_gps_cur_lat = float(data_gps_cur_lat)

            if (data_gps_cur_lon[0] == '-'):
                data_gps_cur_lon = data_gps_cur_lon.replace('-', '')
                data_gps_cur_lon = float(data_gps_cur_lon)
                data_gps_cur_lon = -1 * data_gps_cur_lon
            else:
                data_gps_cur_lon = float(data_gps_cur_lon)

            print(data_gps_cur_lat)
            print(data_gps_cur_lon)
            print(data_gps_cur_height)

            # append to individual lists - target coords
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone3_des_mark.append(data_gps_des_lat)
                data_lon_array_drone3_des_mark.append(data_gps_des_lon)
                data_height_array_drone3_des_mark.append(data_gps_des_height)
            data_lat_array_drone3_des.append(data_gps_des_lat)
            data_lon_array_drone3_des.append(data_gps_des_lon)
            data_height_array_drone3_des.append(data_gps_des_height)

            #append to individual lists - real coords
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone3_cur_mark.append(data_gps_cur_lat)
                data_lon_array_drone3_cur_mark.append(data_gps_cur_lon)
                data_height_array_drone3_cur_mark.append(data_gps_cur_height)
            data_lat_array_drone3_cur.append(data_gps_cur_lat)
            data_lon_array_drone3_cur.append(data_gps_cur_lon)
            data_height_array_drone3_cur.append(data_gps_cur_height)
            data_time_previous = data_time_second[0]
            index += 1

if __name__ == "__main__":
    try:
        main()
        # print(data_lat_array[:])
        # todo: get all plots to show at once instead of having to close one plot
        plt.figure()

        plt.axes(projection='3d')
        plt.plot(data_lat_array_drone2_cur[:], data_lon_array_drone2_cur[:], data_height_array_drone2_cur[:], color='b', linestyle='solid', label='drone2cur')
        #plt.plot(data_lat_array_drone2_cur[:], data_lon_array_drone2_cur[:], 'x')
        plt.plot(data_lat_array_drone2_cur_mark[:], data_lon_array_drone2_cur_mark[:], data_height_array_drone2_cur_mark[:], color='b', marker='o')
        plt.plot(data_lat_array_drone2_des[:], data_lon_array_drone2_des[:], data_height_array_drone2_des[:], color='b', linestyle='dashed', label='drone2des')
        #plt.plot(data_lat_array_drone2_des[:], data_lon_array_drone2_des[:], 'x')
        plt.plot(data_lat_array_drone2_des_mark[:], data_lon_array_drone2_des_mark[:], data_height_array_drone2_des_mark[:], color='b', marker='o')
        plt.title('data plot')
        plt.legend()
        plt.xlabel('data_lat')
        plt.ylabel('data_lon')
        plt.show()

        plt.axes(projection='3d')
        plt.plot(data_lat_array_drone3_cur[:], data_lon_array_drone3_cur[:], data_height_array_drone3_cur[:], color='y', linestyle='solid', label='drone3cur')
        #plt.plot(data_lat_array_drone2_cur[:], data_lon_array_drone2_cur[:], 'x')
        plt.plot(data_lat_array_drone3_cur_mark[:], data_lon_array_drone3_cur_mark[:], data_height_array_drone3_cur_mark[:], color='y', marker='o')
        plt.plot(data_lat_array_drone3_des[:], data_lon_array_drone3_des[:], data_height_array_drone3_des[:], color='y', linestyle='dashed', label='drone3des')
        #plt.plot(data_lat_array_drone2_des[:], data_lon_array_drone2_des[:], 'x')
        plt.plot(data_lat_array_drone3_des_mark[:], data_lon_array_drone3_des_mark[:], data_height_array_drone3_des_mark[:], color='y',marker='o')
        plt.title('data plot')
        plt.legend()
        plt.xlabel('data_lat')
        plt.ylabel('data_lon')
        plt.show()
        time.sleep(5)
    except KeyboardInterrupt:
        print("Interrupted. Exiting program...")
