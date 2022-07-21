# landing_function.py
# Author: Alex Scott

# Largely derived from UC COSC428 Computer Vision Labs

# Make Python look for the right place for pyrealsense2 for the COSC lab PCs.
import sys
sys.path.append('/usr/local/lib') 

# First import the libraries
import pyrealsense2 as rs
import numpy as np
import math
import tkinter as tk



from enum import IntEnum
import pyransac3d as pyrsc

from datetime import datetime
import open3d as o3d

#TO DO: Frame rate or image quality will likely need to be downgraded to operate on the Intel NUC effectively

def pass_through_filter(bbox, pcloud, pres):
    points = np.asarray(pcloud.points)
    colors = np.asarray(pcloud.colors)
    points[:,0]
    x_range = np.logical_and(points[:,0] >= bbox["x"][0] ,points[:,0] <= bbox["x"][1])
    y_range = np.logical_and(points[:,1] >= bbox["y"][0] ,points[:,1] <= bbox["y"][1])
    z_range = np.logical_and(points[:,2] >= bbox["z"][0] ,points[:,2] <= bbox["z"][1])

    enclosed = np.logical_and(x_range,np.logical_and(y_range,z_range))
    

    pres.points = o3d.utility.Vector3dVector(points[enclosed])
    pres.colors = o3d.utility.Vector3dVector(colors[enclosed])

    return pres

def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = o3d.camera.PinholeCameraIntrinsic(640, 480, intrinsics.fx,
                                            intrinsics.fy, intrinsics.ppx,
                                            intrinsics.ppy)
    return out


if __name__ == "__main__":

    # Create a pipeline
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 3  # 3 meter
    clipping_distance = clipping_distance_in_meters / depth_scale
    print(depth_scale)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    
    
     
    

    pcd = o3d.geometry.PointCloud()
    land_box = o3d.geometry.PointCloud()
    dist = o3d.utility.DoubleVector()
    

    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    
    

    # Streaming loop
    frame_count = 0
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                get_intrinsic_matrix(color_frame))

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = o3d.geometry.Image(
                np.array(aligned_depth_frame.get_data()))
            color_temp = np.asarray(color_frame.get_data())
            color_image = o3d.geometry.Image(color_temp)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image,
                depth_image,
                depth_scale=1.0 / depth_scale,
                depth_trunc=clipping_distance_in_meters,
                convert_rgb_to_intensity=False)
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, intrinsic)
            
            temp, indices = temp.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            temp.transform(flip_transform)
            pcd.points = temp.points
            pcd.colors = temp.colors
            
            #Run 50 frames before running the code to avoid start up jitters, this could be much smaller
            if frame_count <= 50:
                if frame_count == 0:
                    vis.add_geometry(pcd)

                vis.update_geometry(pcd)
            
            #Primary function
            if frame_count > 50:
                #Fits a plane to the largest flat surface which is the most likely place for the drone to land
                #Increase threshold for rougher surfaces
                equation, inliers = pyrsc.Plane().fit(pts=np.asarray(pcd.points), thresh=0.05, maxIteration=50)
                [a, b, c, d] = equation
                print(f"pyRANSAC plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} =0")

                #Represent point in the plane and points outside it into seperate groups
                pyransac_plane_pcd = pcd.select_by_index(inliers)
                outlier_cloud = pcd.select_by_index(inliers, invert=True)
                #Can colour the fitted plane green for testing visability
                #pyransac_plane_pcd.paint_uniform_color([0,1,0])  # Green

                #Get the centre of the plane and create a 0.2 x 0.2 metre box around it to represent the drone
                #The real drone will be larger
                cent = o3d.geometry.PointCloud.get_center(pyransac_plane_pcd)                
                hgt = abs(cent[2]*c)
                distx = abs(cent[0])
                disty = abs(cent[1])
                xcoordl = (distx-0.1)
                xcoordh = (distx+0.1)
                ycoordl = (disty-0.1)
                ycoordh = (disty+0.1)
                angle_bcos = 1/((a**2+b**2+1)**(1/2))
                angle_rad = math.acos(angle_bcos)
                angle = angle_rad*180/3.14159
                print(angle)
                print(cent)

                #Prints height and angle data for testing
                print(f"Drone is {hgt:.2f} metres high")
                print(f"Drone is at a {angle:.2f} degree angle")
                

                #Create the landing box for the drone
                bounding_box = {"x":[xcoordl,xcoordh], "y":[ycoordl,ycoordh], "z":[-3.0,3.0]}
                land_box = pass_through_filter(bounding_box, pyransac_plane_pcd, land_box)
                land_box.paint_uniform_color([0,0,1])
                temp = land_box + outlier_cloud + pyransac_plane_pcd
                temp, indices = temp.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                pcd.points = temp.points
                pcd.colors = temp.colors
                
                #TODO: Compare the X and Y distance between the landing box and outlier cloud to determine
                # interfering objects. Previous attempts use compute_point_cloud_distance, this would also
                # account for the Z distance making it unable to register obstacles that had too much vertical
                # distance from the landing box
                
                
            vis.update_geometry(pcd)    
            vis.poll_events()
            vis.update_renderer() 
                
            frame_count += 1

    finally:
        pipeline.stop()
    vis.destroy_window()