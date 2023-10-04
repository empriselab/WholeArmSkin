#! /usr/bin/env python 

import rospy
# Other imports
import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import random
import tf
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import json
# Program Code


#configurations go from bottom to top

#6-configuration
shoulder_link = [[13.964,44.504,198.851, 90, 0, 90], [-13.978, 44.373, 198.945, 89.037, 0, 90],[15.138,45.408,249.34, 86.53,0,90],
[-15.254,45.412,249.575, 86.514, 0, 90], [15,47.612,294.401,90.7, 0, 90], [-14.998, 47.613, 293.402, 90.655, 0, 90]]

#7-configuration
half_arm_1_link = [[0, -59.434, 260.438, -90, 180, 90], [-20.672, -57.692, 304.084, -90, 180, 90], [20.673, -57.702, 304.086, -90, 180, 90], 
[-19.573, -54.952, 348.666, -90, 180, 90], [19.843, -54.885, 348.823, -90, 180, 90], [-18.162, -52.60, 394.471, -90, 180, 90], 
[18.181, -52.619, 394.175, 90, 180, 90],

#ring configuration
[0, -56.428, 440.124, 180, 180, 90], [36.757, -37.739, 440.124, 180, 180, 150], [36.147, 4.352, 440.124, 180, 180, 210], 
[.047, 23.07, 440.023, 180, 180, 270], [-35.955, 4.276, 440.124, 180, 180, 330], [-37.018, -37.855, 440.124, 180, 180, 30],

#ring configuration
[0.041, -57.747, 484.514, 90.00, 190, 90], [39.956, -34.544, 484.514, 90 ,190 , 150], [39.719, 11.454, 484.514, 90, 180, 210], 
[0,34.25, 484.495, 90, 180, 270], [-39.671, 11.535, 484.514, 90, 180, 330], [-40.002, -34.461, 484.514, 90, 190, 30]]

half_arm_2_link = [[.02, -57.016, 526.302, 101.598, 180, 90],[39.129, -34.183, 526.677, 96.579, 180, 150], [39.217, 11.022, 526.982, 92.981, 180, 210], 
 [0.05, 33.731, 527.23, 91.859, 180, 270], [-39.167, 11.113, 526.982, -92.982, 180, 330], [-39.167, -34.081, 526.673, 96.605, 180, 30],
 
 [8.739, 31.523, 570.829, 91.206, 0, 90], [-8.844, 31.499, 570.829, 91.207, 0, 90], [10.069, 32.338, 622.447, 86.715, 0, 90], 
 [-9.895, 32.332, 622.08, 86.73, 0, 90], [11.872, 34.638, 667.549, 86.336, 0, 90], [-11.701, 34.645, 667.449, 86.336, 0, 90], 
 [0, 37.252, 707.728, 89.86, 0, 90]]

forearm_link = [[0, -73.572, 691.351, 85.557, 0, 270], [-14.701, -72.926, 725.332, 93.521, 0, 270], [14.749, -72.921, 725.327, 93.523, 0, 270],
  [-11.652, -69.509, 767.149, 95.538, 0, 270], [11.772, -69.488, 767.163, 95.547, 0, 270], [-8.496, -65.496, 809.544, 97.087, 0, 270], 
  [8.412, -65.50, 809.412, 82.932, 0, 270],
  
  [-.005, -62.5, 858.469, 94.439, 180, 90], [29.40, -44.479, 858.469, 92.926, 180, 150], [27.66, -11.536, 858.469, 74.875, 160, 210], 
  [0, 3.741, 858.469, 66.96, 160, 270], [-27.68, -11.514, 858.469, 74.875, 160, 330], [-29.418, -44.479, 858.469, 92.924, 180, 30],
  
  [-.044, -59.498, 909.882, 90, 180, 90], [30.059, -42.429, 909.90, 90, 180, 150], [30.56, -7.439, 909.90, 90, 180, 210],
   [0, 10.5, 909.9, 90, 180, 270], [-30.464, -7.268, 909.9, 90 , 180, 330], [-30.157, -42.263, 909.9, 90, 180, 30]]

spherical_wrist_1_link = [[0, -59.111, 945.212, 96.617,190, 90], [29.488, -41.053, 945.212, 99.922, 200, 150], [29.948, -6.737, 945.212, 81.286, 180, 210], 
   [0, 11.14, 945.238, 84.985, 180, 270], [-29.582, -6.737, 945.212, 81.259, 180, 330], [-29.852, -41.207, 945.212, 81.259, 200, 30],
   
   [14.885, 22.245, 1000.345, 73.447, 20.489, 78], [-14.885, 22.245, 1000.345, 73.447, 20.489, 102] , [0, 28.276, 1030.399, 98.374, 0, 90]]

#spherical_wrist_2_link = [[0, -77.824, 1010.989, 84.141, 0, 270], [-14.146, -71.622, 1039.875, 106.657, -20.176, 258], [14.131, -71.624, 1039.876, 106.657, -20.176, 282],
#   
#   [.007, -61.50, 1091.835, 95.965, 5.965, 270], [30.306, -42.989, 1091.835, 100.465, 18, 330], [29.084, -8.7, 1091.835, 74.562, 18, 30], 
#   [-.004, 9.295, 1091.835, 78.054, 11.946, 90], [-29.111, -8.7, 1091.835, 74.579, 18, 150], [-30.278, -42.989, 1091.835, 100.476, 18, 210]]

spherical_wrist_2_link = [[17.532, 5.378, 1131.382, 90, 0, 60],
[35.075, -24.994, 1131.382, 90, 0, 0],
[17.542, -55.373, 1131.382, 90, 0, 300],
[14.131, -71.624, 1039.876, 106.657, -20.176, 282],
[-17.532, -55.378, 1131.382, 90, 0, 240],
[-14.146, -71.622, 1039.875, 106.657, -20.176, 258],
[-35.075, -25.006, 1131.382, 90, 0, 180],
[-17.542, 5.373, 1131.382, 90, 0, 120]],
[0, -77.824,1010.989,84.141,0,270]



link_names = ['shoulder_link', 'half_arm_1_link', 'half_arm_2_link', 'forearm_link', 'spherical_wrist_1_link', 'spherical_wrist_2_link']
robot = dict()

for e in range(len(link_names)):
    for i, pose in enumerate(eval(link_names[e])):
        new_pos = PoseStamped()
        new_pos.header.frame_id = link_names[e]
        new_pos.pose.position.x = pose[0]/1000
        new_pos.pose.position.y = pose[1]/1000
        new_pos.pose.position.z = pose[2]/1000
        quat = tf.transformations.quaternion_from_euler(np.radians(pose[3]), np.radians(pose[4]), np.radians(pose[5]))
        new_pos.pose.orientation.x = quat[0]
        new_pos.pose.orientation.y = quat[1]
        new_pos.pose.orientation.z = quat[2]
        new_pos.pose.orientation.w = quat[3]
        eval(link_names[e])[i] = new_pos

#initalize listener node
rospy.init_node('robot_listener')
tf2_buffer = tf2_ros.Buffer()
tf2_listener = tf2_ros.TransformListener(tf2_buffer)

tran = None

for e in range(len(link_names)):
    while(True):
        try:
            tran = tf2_buffer.lookup_transform(link_names[e], "base_link", rospy.Time())
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            SystemError("Transform not found")

    #tran.transform.translation.x = tran.transform.translation.x
    #tran.transform.translation.y = tran.transform.translation.y
    #tran.transform.translation.z = tran.transform.translation.z
    robot[link_names[e]] = []
    for i in range(len(eval(link_names[e]))):    
        transformed_pos = do_transform_pose(eval(link_names[e])[i], tran)
        robot[link_names[e]].append([transformed_pos.pose.position.x, transformed_pos.pose.position.y, transformed_pos.pose.position.z, transformed_pos.pose.orientation.x, transformed_pos.pose.orientation.y, transformed_pos.pose.orientation.z, transformed_pos.pose.orientation.w])

    #save robot to json
    with open('/home/emprise/wholearm_ws/wholearm_ws/robot.json', 'w') as fp:
        json.dump(robot, fp) 