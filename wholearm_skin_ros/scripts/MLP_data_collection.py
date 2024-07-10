#!/usr/bin/env python3

import rospy
from wholearm_skin_ros.msg import TaxelData, TaxelPosOri
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
import message_filters
from collections import deque
from scipy import signal
from digitalfilter import LiveSosFilter
import pickle
import numpy as np
import threading

# Declaring a lock
lock = threading.Lock()

taxel = 13
filename = 'test'

from std_srvs.srv import SetBool, SetBoolResponse

class DataCollection:
    def __init__(self) -> None:
        rospy.init_node('data_collection', anonymous=True, disable_signals=True)
        s = rospy.Service('data_collection/reset_skin_bias', SetBool, self.tare_skin_data)
        self.joint_sub = message_filters.Subscriber("/joint_states", JointState)
        # self.tf_sub = message_filters.Subscriber("/tf_header", TFHeader)
        # self.tf_static_sub = message_filters.Subscriber("/tf_static", TFMessage)
        self.markers_sub = message_filters.Subscriber("/taxelposori", TaxelPosOri)
        self.ft_sub = message_filters.Subscriber("/ft_sensor/netft_data", WrenchStamped)
        self.skin_sub = message_filters.Subscriber("/skin/taxels", TaxelData)
        self.sync_sub = message_filters.ApproximateTimeSynchronizer([self.ft_sub, self.skin_sub, self.joint_sub, self.markers_sub],
                                                               queue_size=10, slop=0.1)
        self.pub = rospy.Publisher("/mlp_data", Float32, queue_size=10)
        self.sync_sub.registerCallback(self.sync_callback)
        self.calibration_data = {'ft': [], 'skin': [], 'ft_time': [], 'skin_time': [], 'joint_pos': [], 'joint_vel': [], 'joint_effort': [], 
                                'joint_time': [], 'taxel_pos': [], 'taxel_ori': [], 'taxel': []}

        self.skin_history = deque(maxlen=100)
        self.tare_value = 0
        self.taring = False
        self.prev_val = 0

        # Filter variables
        # TODO: Change filter parameters
        sos = signal.iirfilter(2, Wn=1.5, fs=30, btype="lowpass",
                             ftype="butter", output="sos")
        self.live_sosfilter = LiveSosFilter(sos)
        self.filter_count = 0

        # add magnitude filter: maybe just use this for non-conductive material
        self.magnitude_filter = False
        # self.max_value = 1000

    def tare_skin_data(self, req):
        if req.data is True:
            self.taring = False
            self.skin_history.clear()
            self.tare_value = 0
            self.filter_count = 0
            self.calibration_data['ft'].clear()
            self.calibration_data['skin'].clear()
            self.calibration_data['ft_time'].clear()
            self.calibration_data['skin_time'].clear()
            return SetBoolResponse(True, "Started taring process")

    def filter(self, skin_data):
        val = self.live_sosfilter(skin_data)
        if self.magnitude_filter and self.taring:
            if skin_data > self.max_value:
                return self.max_value
        return val

    def sync_callback(self, ft_msg, skin_msg, joint_msg, marker_msg):
        if len(self.skin_history) == self.skin_history.maxlen and not self.taring and self.filter_count > 100:
            self.taring = True
            print('taring TRUE')
            self.tare_value = int(sum(self.skin_history)/len(self.skin_history))
            self.filter_count = 0

        self.filter_count += 1
        print(self.filter_count)
        skin_data = self.filter(skin_msg.cdc[taxel] - self.tare_value)
        # skin_data = skin_msg.cdc[taxel] - self.tare_value

        if not self.taring:
            self.skin_history.append(skin_data)
        elif self.filter_count > 200:
            msg = Float32()
            # msg.data = skin_data
            self.pub.publish(msg)
            ft_norm = -np.sqrt(ft_msg.wrench.force.x**2 + ft_msg.wrench.force.y**2 + ft_msg.wrench.force.z**2)
            print("FT: ", ft_norm, " Skin: ", skin_data, "Time diff: ", ft_msg.header.stamp - skin_msg.header.stamp)
            self.store_data([ft_msg.wrench.force.x, ft_msg.wrench.force.y, ft_msg.wrench.force.z], skin_data, ft_msg.header.stamp.to_sec(), skin_msg.header.stamp.to_sec(), joint_msg.position, 
                            joint_msg.velocity, joint_msg.effort, joint_msg.header.stamp.to_sec(), marker_msg.position, marker_msg.orientation)

    def store_data(self, ft_data, skin_data, ft_time, skin_time, joint_pos, joint_vel, joint_effort, joint_time, marker_pos, marker_ori):
        self.calibration_data['ft'].append(ft_data)
        self.calibration_data['skin'].append(skin_data)
        self.calibration_data['ft_time'].append(ft_time)
        self.calibration_data['skin_time'].append(skin_time)
        self.calibration_data['joint_pos'].append(joint_pos)
        self.calibration_data['joint_vel'].append(joint_vel)
        self.calibration_data['joint_effort'].append(joint_effort)
        self.calibration_data['joint_time'].append(joint_time)
        # self.calibration_data['taxel_markers'].append(taxel_markers)
        # self.calibration_data['tf'].append(tf)
        # self.calibration_data['tf_static'].append(tf_static)
        self.calibration_data['taxel_pos'].append(marker_pos)
        self.calibration_data['taxel_ori'].append(marker_ori)
        self.calibration_data['taxel'].append(taxel)

    def save_data(self):
        rospy.loginfo('Saving data') 
        with open("data_collection_" + filename + ".pickle", 'wb') as handle: 
            pickle.dump(self.calibration_data, handle, protocol=pickle.HIGHEST_PROTOCOL)

def callback(original_msg):
    modified_msg = TaxelPosOri()
    # Copy specific taxel fields to the root level
    modified_msg.header = original_msg.markers[taxel].header
    modified_msg.id = original_msg.markers[taxel].id
    modified_msg.position = original_msg.markers[taxel].pose.position
    modified_msg.orientation = original_msg.markers[taxel].pose.orientation

    # Publish the modified message
    taxelposori_pub.publish(modified_msg)

if __name__ == "__main__":
    data_collector = DataCollection()
    lock.acquire()
    try:
        rospy.loginfo("Starting data collection")
        # Modifying message
        rospy.Subscriber('/taxel_markers', MarkerArray, callback)
        taxelposori_pub = rospy.Publisher('/taxelposori', TaxelPosOri, queue_size=10)
        rospy.spin()
    finally:
        data_collector.save_data()
