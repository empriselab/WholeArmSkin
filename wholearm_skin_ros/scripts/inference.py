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

num_taxels = 37  # TODO edit this to match

model = \
[
 # link 1
 0, 0, 0, 0, 0, # link1_0 bad taxel
 3.7220725284896235e-11, -8.932159686395518e-08, 7.274698239019916e-05, -0.006357885342385423, 0, # link1_1
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_2
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_3
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_4
 2.656010906120618e-11, -7.930896334754945e-08, 7.87772539827809e-05, -0.012701875093228095, 0, # link1_5
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_6
 0, 0, 0, 0, 0, # link1_7 bad taxel
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link1_8
 # link 2
 6.6961583624802546e-12, -1.9630251828391848e-08, 2.564118799568668e-05, -0.007545461378169108, 0, # link2_9
 4.082930054884611e-13, -3.1204772940760058e-09, 4.846646008141072e-06, 0.005914403433672906, 0, # link2_10
 -3.5298489660063897e-12, 1.1978324944902293e-08, -1.2104368380348903e-05, 0.011207918478242757, 0, # link2_11
 0, 0, 0, 0, 0, # link2_12 bad taxel
#  -2.8829151748050613e-12, 1.0134411351636654e-08, -1.1322262648551302e-05, 0.01136516852478394, 0, # link2_13
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link2_13
 -2.2087998057256407e-11, 5.8712373857854355e-08, -4.8096423470894806e-05, 0.021319601816394338, 0, # link2_14
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link2_15
 4.6054043479980093e-11, -1.0241385695471127e-07, 7.735736819387364e-05, -0.01159500216627978, 0, # link2_16
 2.9030929546135403e-11, -6.76278981794175e-08, 6.196869722788381e-05, -0.012586809143708966, 0, # link2_17
 # link 3
 1.4249697972649956e-12/25, -1.1175133281183478e-08/25, 2.8066729292306418e-05/25, -0.005112112703314263/25, 0, # link3_18
 1.11350321071551e-12*0.3, -1.0484860758794027e-08*0.3, 1.6615278314679865e-05*0.3, 0.006837914945215908*0.3, 0, # link3_19
 -2.846759134249069e-14*4, 4.872959135792518e-10*4, -2.7101749258893633e-06*4, 0.0070905462057438305*4, 0, # link3_18
 -2.5603251614902814e-14*3, 4.123683201239358e-10*3, -2.200926956575998e-06*3, 0.005814835234170356*3, 0, # link3_21
 -2.846759134249069e-14*4, 4.872959135792518e-10*4, -2.7101749258893633e-06*4, 0.0070905462057438305*4, 0, # link3_22
 -7.97795568647039e-14*4, 1.114307493674375e-09*4, -5.3262779009338445e-06*4, 0.011302242334681769*4, 0, # link3_23
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link3_24
 0,0,0,0,0, # link3_25 bad taxel
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link3_26 same as link1_6
 0,0,0,0,0, # link3_27 bad taxel
 -2.0809270946816543e-13*3, 3.286419713409457e-09*3, -1.5903194547216634e-05*3, 0.024199589589892746*3, 0, # link3_28
 1.4249697972649956e-12/10, -1.1175133281183478e-08/10, 2.8066729292306418e-05/10, -0.005112112703314263/10, 0, # link3_29
 1.7499790892708405e-12, -7.77446644488818e-09, 9.146093110210723e-06, 0.003289257986211262, 0, # link3_30
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link3_31
 0,0,0,0,0, # link3_32 bad taxel
 -2.3122436258460522e-12, 4.945841041530434e-09, 3.8656797154457675e-06, 0.004380627928835595, 0, # link3_33
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, -26, # link3_34
 1.4249697972649956e-12, -1.1175133281183478e-08, 2.8066729292306418e-05, -0.005112112703314263, 0, # link3_35
 1.4249697972649956e-12*0.5, -1.1175133281183478e-08*0.5, 2.8066729292306418e-05*0.5, -0.005112112703314263*0.5, 0, # link3_36
]
model = np.array(model).reshape(37,5)


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
                if (i == 19):
                    skin_data[i] = skin_msg.cdc[i] - 230
            #     skin_data[i] = self.filter(skin_msg.cdc[i] - self.tare_value[i], self.live_sosfilter)
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
                if (msg.data[i] < 0.5):
                    msg.data[i] = 0
                print(msg.data[20])
                # temporary:
                if (msg.data[i] > 60):
                    msg.data[i] = 60
                if (msg.data[9] > 1):
                    msg.data[3] = 0
                    msg.data[4] = 0
                    msg.data[24] = 0
                    msg.data[31] = 0
                    msg.data[35] = 0
                if (msg.data[10] > 1):
                    msg.data[23] = 0
                    msg.data[29] = 0
                    msg.data[1] = 0
                    msg.data[2] = 0
                    msg.data[6] = 0
                if (msg.data[29] == 60):
                    msg.data[29] = 0
                
            self.pub.publish(msg)
            print(msg.data[0])
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
