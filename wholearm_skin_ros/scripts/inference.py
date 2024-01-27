#!/usr/bin/env python3

import rospy
from wholearm_skin_ros.msg import TaxelData
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import message_filters
from collections import deque
from scipy import signal
from digitalfilter import LiveSosFilter
import pickle
import scipy.linalg
import matplotlib.pyplot as plt
import numpy as np
from numpy_ringbuffer import RingBuffer

import threading
# Declraing a lock
lock = threading.Lock()

num_taxels = 18  # TODO edit this to match

model = \
[0, 0, 0, 0, 0, # link1_0 bad taxel
 3.7220725284896235e-11, -8.932159686395518e-08, 7.274698239019916e-05, -0.006357885342385423, 0, # link1_1
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_2
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_3
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_4
 2.656010906120618e-11, -7.930896334754945e-08, 7.87772539827809e-05, -0.012701875093228095, 0, # link1_5
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_6
 0, 0, 0, 0, 0, # link1_7 bad taxel
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_8
 6.6961583624802546e-12, -1.9630251828391848e-08, 2.564118799568668e-05, -0.007545461378169108, 0, # link2_9
 4.082930054884611e-13, -3.1204772940760058e-09, 4.846646008141072e-06, 0.005914403433672906, 0, # link2_10
 -3.5298489660063897e-12, 1.1978324944902293e-08, -1.2104368380348903e-05, 0.011207918478242757, 0, # link2_11
 0, 0, 0, 0, 0, # link2_12 bad taxel
#  -2.8829151748050613e-12, 1.0134411351636654e-08, -1.1322262648551302e-05, 0.01136516852478394, 0, # link2_13
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_13
 -2.2087998057256407e-11, 5.8712373857854355e-08, -4.8096423470894806e-05, 0.021319601816394338, 0, # link2_14
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link2_15
 4.6054043479980093e-11, -1.0241385695471127e-07, 7.735736819387364e-05, -0.01159500216627978, 0, # link2_16
 2.9030929546135403e-11, -6.76278981794175e-08, 6.196869722788381e-05, -0.012586809143708966, 0 # link2_17
 ]
model = np.array(model).reshape(18,5)


class DataCollection:
    def __init__(self) -> None:
        rospy.init_node('calibration', anonymous=True, disable_signals=True)
        self.skin_sub = rospy.Subscriber(
            "/skin/taxel_fast", TaxelData, self.callback)
        self.pub = rospy.Publisher(
            "/calibration", Float32MultiArray, queue_size=10)
        # self.model = pickle.load(
        #     open("/home/zly/wholearm_ws/fitted_poly3_model_0427_05.pickle", "rb"))
        # Taring variables
        # TODO: max length should be a parameter
        self.taring = False
        # ring buffer with capacity 100, extra dimesions (num_taxels) for list of taxel readings
        self.skin_history = RingBuffer(
            capacity=100, dtype=(float, (num_taxels)))
        self.tare_value = [0] * num_taxels
        self.last_value = [0] * num_taxels

        # Filter variables
        # TODO: Change filter parameters
        sos = signal.iirfilter(
            2, Wn=1.5, fs=60, btype="lowpass", ftype="butter", output="sos")
        # self.live_sosfilter = [ LiveSosFilter(sos), LiveSosFilter(sos), LiveSosFilter(sos), LiveSosFilter(sos), \
        #                         LiveSosFilter(sos), LiveSosFilter(sos), LiveSosFilter(sos), LiveSosFilter(sos) ]
        # self.live_sosfilter = [ LiveSosFilter(sos) ] * num_taxels

        self.live_sosfilter = []
        for i in range(num_taxels):
            self.live_sosfilter.append(LiveSosFilter(sos))

    def callback(self, skin_msg):
        if self.skin_history.is_full and not self.taring:
            self.taring = True
            for i in range(num_taxels):
                self.tare_value[i] = int(
                    sum(self.skin_history[:, i]))/len(self.skin_history[:, i])

        if not self.taring:
            # print(np.shape(np.array(skin_msg.cdc)))
            # print(skin_msg.cdc)
            # print(np.array(skin_msg.cdc).reshape(1,2))
            print("Taring...")
            self.skin_history.extend(
                np.array(skin_msg.cdc).reshape(1, num_taxels))
            # print(np.shape(self.skin_history))
        else:
            skin_data = np.zeros(num_taxels)
            # print("///////////////")
            for i in range(num_taxels):
                # print(i)
                # print(skin_msg.cdc[i])
                # print("skin data size: ")
                # print(len(skin_data[i]))
                #### filter data processing
                # filtered = self.filter(
                #     skin_msg.cdc[i] - self.tare_value[i], self.live_sosfilter[i])
                # print(filtered)
                # print(len(filtered))
                # skin_data[i] = filtered
                if (i != 5):
                    skin_data[i] = skin_msg.cdc[i] - self.tare_value[i]
                if (i == 5):
                    skin_data[i] = self.tare_value[i] - skin_msg.cdc[i]
                # skin_data[i] = self.filter(skin_msg.cdc[i] - self.tare_value[i], self.live_sosfilter)
            # print(skin_data)

            msg = Float32MultiArray()
            msg.data = [0] * num_taxels
            self.last_value = skin_data

            # fit data to calibration model
            for i in range(num_taxels):
                # msg.data[i] = skin_data[i] * self.model['m'] + self.model['c']
                # msg.data[i] = self.model['a3'] + self.model['a2']*skin_data[i] + \
                #     self.model['a1']*skin_data[i]**2 + \
                #     self.model['a0']*skin_data[i]**3
                msg.data[i] = model[i][0]*skin_data[i]**4 + \
                            model[i][1]*skin_data[i]**3 + \
                            model[i][2]*skin_data[i]**2 + \
                            model[i][3]*skin_data[i] + model[i][4]
                if (skin_data[i] < 0):
                    msg.data[i] = 0
                if (msg.data[i] < 0):
                    msg.data[i] = 0
                print(msg.data[15])

                # temporary:
                if (msg.data[9] > 2):
                    msg.data[4] = 0
                if (msg.data[10] > 2):
                    msg.data[1] = 0
                    msg.data[2] = 0
                    msg.data[6] = 0
                # msg.data[i] = skin_data[i]
            # msg.data = skin_data * self.model['m'] + self.model['c']
            # msg.layout = MultiArrayLayout()
            # msg.layout.dim = (MultiArrayDimension())
            # msg.layout.dim.push_back(MultiArrayDimension())
            # msg.layout.dim.size = num_taxels
            # msg.layout.dim.stride = 1
            # msg.layout.data_offset = 0\
            # print(self.model['a3'])
            # print(self.model['a2'])
            # print(self.model['a1'])
            self.pub.publish(msg)
            # print(msg.data)
            # with open("/home/emprise/wholearm_ws/data.txt", "a") as f:
            #     f.write(str(msg.data) + "\n")
            

    def filter(self, skin_data, filter_fn):
        return filter_fn(skin_data)


if __name__ == "__main__":
    data_collector = DataCollection()
    lock.acquire()
    # try:
    rospy.loginfo("Starting inference data collection")
    rospy.spin()
    # finally:
    # data_collector.save_data()
