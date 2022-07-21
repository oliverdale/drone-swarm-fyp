import string
import matplotlib.pyplot as plt
import time
import numpy as np

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

data_lat_array_drone1_cur, data_lon_array_drone1_cur = [], []
data_lat_array_drone1_cur_mark, data_lon_array_drone1_cur_mark = [], []
data_lat_array_drone1_des, data_lon_array_drone1_des = [], []
data_lat_array_drone1_des_mark, data_lon_array_drone1_des_mark = [], []

data_lat_array_drone2_cur, data_lon_array_drone2_cur = [], []
data_lat_array_drone2_cur_mark, data_lon_array_drone2_cur_mark = [], []
data_lat_array_drone2_des, data_lon_array_drone2_des = [], []
data_lat_array_drone2_des_mark, data_lon_array_drone2_des_mark = [], []






def main():
    index = 0
    data_time_previous = None
    data_cur = log_file_drone1_CUR_name
    data_temp_cur = open(data_cur, "r")

    data_ff_cur_1 = data_temp_cur.readline()
    data_ff_cur_1 = data_temp_cur.readlines()

    data_des = log_file_drone1_DES_name
    data_temp_des = open(data_des, "r")

    data_ff_des_1 = data_temp_des.readline()
    data_ff_des_1 = data_temp_des.readlines()

    for line in data_ff_cur_1:
        if (line == ' '):
            continue

        data_array = line.strip().split(" ")
        data_time = str(data_array[0])
        if ((data_time[9] != data_time_previous) or (data_time_previous == None)):
            data_day = data_array[1]
            data_mnth = data_array[2]
            data_date = data_array[4]
            data_year = data_array[5]
            data_year = data_year.replace(':', '')
            data_lat = data_array[6]
            data_lat = data_lat.replace(',', '')
            data_lon = data_array[7]
            if (data_lat[0] == '-'):
                data_lat = data_lat.replace('-', '')
                data_lat = float(data_lat)
                data_lat = -1 * data_lat
            else:
                data_lat = float(data_lat)

            if (data_lon[0] == '-'):
                data_lat = data_lon.replace('-', '')
                data_lon = float(data_lon)
                data_lon = -1 * data_lon
            else:
                data_lon = float(data_lon)

            print(data_lat)
            print(data_lon)
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone1_cur_mark.append(data_lat)
                data_lon_array_drone1_cur_mark.append(data_lon)
            data_lat_array_drone1_cur.append(data_lat)
            data_lon_array_drone1_cur.append(data_lon)
            data_time_previous = data_time[9]
            index += 1

    data_time_previous = None
    index = 0
    for line in data_ff_des_1:
        if (line == ' '):
            continue
        
        data_array = line.strip().split(" ")
        data_time = str(data_array[0])
        if ((data_time[9] != data_time_previous) or (data_time_previous == None)):
            data_day = data_array[1]
            data_mnth = data_array[2]
            data_date = data_array[4]
            data_year = data_array[5]
            data_year = data_year.replace(':', '')
            data_lat = data_array[6]
            data_lat = data_lat.replace(',', '')
            data_lon = data_array[7]
            if (data_lat[0] == '-'):
                data_lat = data_lat.replace('-', '')
                data_lat = float(data_lat)
                data_lat = -1 * data_lat
            else:
                data_lat = float(data_lat)

            if (data_lon[0] == '-'):
                data_lat = data_lon.replace('-', '')
                data_lon = float(data_lon)
                data_lon = -1 * data_lon
            else:
                data_lon = float(data_lon)

            print(data_lat)
            print(data_lon)
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone1_des_mark.append(data_lat)
                data_lon_array_drone1_des_mark.append(data_lon)
            data_lat_array_drone1_des.append(data_lat)
            data_lon_array_drone1_des.append(data_lon)
            data_time_previous = data_time[9]
            index += 1

    data_time_previous = None
    index = 0
    data_cur = log_file_drone2_CUR_name
    data_temp_cur = open(data_cur, "r")

    data_ff_cur_2 = data_temp_cur.readline()
    data_ff_cur_2 = data_temp_cur.readlines()

    data_des = log_file_drone2_DES_name
    data_temp_des = open(data_des, "r")

    data_ff_des_2 = data_temp_des.readline()
    data_ff_des_2 = data_temp_des.readlines()

    for line in data_ff_cur_2:
        if (line == ' '):
            continue

        data_array = line.strip().split(" ")
        data_time = str(data_array[0])
        if ((data_time[9] != data_time_previous) or (data_time_previous == None)):
            data_day = data_array[1]
            data_mnth = data_array[2]
            data_date = data_array[4]
            data_year = data_array[5]
            data_year = data_year.replace(':', '')
            data_lat = data_array[6]
            data_lat = data_lat.replace(',', '')
            data_lon = data_array[7]
            if (data_lat[0] == '-'):
                data_lat = data_lat.replace('-', '')
                data_lat = float(data_lat)
                data_lat = -1 * data_lat
            else:
                data_lat = float(data_lat)

            if (data_lon[0] == '-'):
                data_lat = data_lon.replace('-', '')
                data_lon = float(data_lon)
                data_lon = -1 * data_lon
            else:
                data_lon = float(data_lon)

            print(data_lat)
            print(data_lon)
            if ((index == 0) or (index % 5 == 0)):
                data_lat_array_drone2_cur_mark.append(data_lat)
                data_lon_array_drone2_cur_mark.append(data_lon)
            data_lat_array_drone2_cur.append(data_lat)
            data_lon_array_drone2_cur.append(data_lon)
            data_time_previous = data_time[9]
            index += 1



    data_time_previous = None
    index = 0

    for line in data_ff_des_2:
        if (line == ' '):
            continue

        data_array = line.strip().split(" ")
        data_time = str(data_array[0])
        if ((data_time[9] != data_time_previous) or (data_time_previous == None)):
            data_day = data_array[1]
            data_mnth = data_array[2]
            data_date = data_array[4]
            data_year = data_array[5]
            data_year = data_year.replace(':', '')
            data_lat = data_array[6]
            data_lat = data_lat.replace(',', '')
            data_lon = data_array[7]
            if (data_lat[0] == '-'):
                data_lat = data_lat.replace('-', '')
                data_lat = float(data_lat)
                data_lat = -1 * data_lat
            else:
                data_lat = float(data_lat)

            if (data_lon[0] == '-'):
                data_lat = data_lon.replace('-', '')
                data_lon = float(data_lon)
                data_lon = -1 * data_lon
            else:
                data_lon = float(data_lon)

            print(data_lat)
            print(data_lon)
            if ((index == 0) or (index % 5 == 0)):
                data_lon_array_drone2_des_mark.append(data_lat)
                data_lat_array_drone2_des_mark.append(data_lon)
            data_lat_array_drone2_des.append(data_lat)
            data_lon_array_drone2_des.append(data_lon)
            data_time_previous == data_time[9]
            index += 1


if __name__ == "__main__":
    try:
        main()
        # print(data_lat_array[:])
        plt.axes(projection='3d')
        plt.plot(data_lat_array_drone1_cur[:], data_lon_array_drone1_cur[:], color='b', label='drone1cur')
        plt.plot(data_lat_array_drone1_cur[:], data_lon_array_drone1_cur[:], 'x')
        plt.plot(data_lat_array_drone1_cur_mark[:], data_lon_array_drone1_cur_mark[:], 'o')
        plt.plot(data_lat_array_drone1_des[:], data_lon_array_drone1_des[:], color='y', label='drone1des')
        plt.plot(data_lat_array_drone1_des[:], data_lon_array_drone1_des[:], 'x')
        plt.plot(data_lat_array_drone1_des_mark[:], data_lon_array_drone1_des_mark[:], 'o')
        #plt.plot(data_lat_array_drone2_cur[:], data_lon_array_drone2_cur[:], color='r', label='drone2cur')
        #plt.plot(data_lat_array_drone2_cur_mark[:], data_lon_array_drone2_cur_mark[:], 'o')
        #plt.plot(data_lat_array_drone2_des[:], data_lon_array_drone2_des[:], color='c', label='drone2des')
        #plt.plot(data_lat_array_drone2_des_mark[:], data_lon_array_drone2_des_mark[:], 'o')
        plt.title('data plot')
        plt.legend()
        plt.xlabel('data_lat')
        plt.ylabel('data_lon')
        plt.show()
        time.sleep(5)
    except KeyboardInterrupt:
        print("Interrupted. Exiting program...")
