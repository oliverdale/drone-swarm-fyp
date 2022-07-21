The followig log files are from UAV flight tests.

Test 1 - 1 flying drone, 4 simulated:
Drones 0,2,3,4 were simulated. Drone 1 was flying
The scripts were run, as per practical flight test in the main READMD
The drone was manually flown to a position, then switched to OFFBOARD mode, where it successfully moved in formation.
Once the test was successfully completed, the drones were switched into position control and landed.
The scripts (and data logging) stopped 3 seconds after the change to position control.

Test 2 part 1 - 2 flying drones, 3 simulated:
Drones 0,2,3 were simulated. Drones 1,4 were flying
Bothe UAVs were flown into position, then switched to OFFBOARD mode.
The drones initially held their formation correctly.
They then had an error of an unsafe formation, the cause of which is unknown. 
The drones appeared to be holding formation correctly - perhaps the error was due to a false GPS reading?
The result of the error was that the drones stopped tracking and stayed at their position, until they were manually landed.
The scripts (and data logging) stopped 3 seconds after the change to position control.

Test 2 part 2 - 2 flying drones, 3 simulated:
THERE IS NO LINE SPACE BETWEEN PART 1 AND 2 IN THE LOG FILES.
The 2 drone flight was again tested, with a similar error as above.
The error occured sooner, so this test wa shorter than part 1.


Drone formation
1          2

     0

3          4



Log files:

drone_positions.txt:
Containes the GPS location of the drones.
The coordinateds are in the order 0,1,2,3,4

estimated_target_positions.txt
This contains the target position, and its estimate
The latitude and longitude for true and estimated are wrong, e.g. for the following line the estimated target is (-43.522374493216056, 172.57779770525835) and the true position is (-43.52237180234548, 172.57779)
time 1605568554.1028259, target estimate: -43.522374493216056,-43.52237180234548 true target 172.57779,172.57779770525835 target error: 0.6896
The python line writing to the file:
log_file.write("time {}, target estimate: {},{} true target {},{} target error: {:.4}\n".format(
                last_updates_received_time, target_coords.lat, self.updates.get_actual_target_coords().lat, 
                self.updates.get_actual_target_coords().long, target_coords.long, multilat_error))

target_positions.txt:
This is the GPS output, which should be the same as the true target position in estimated_target_positions.txt
This file is longer, as it was running before and after the drone tests
