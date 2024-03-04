#!/usr/bin/env python3

import pickle
import scipy.linalg
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import sys, getopt
import statistics

filename = "link1_1"
data = pickle.load(open("data_collection_" + filename + ".pickle", "rb"))
skin = np.array(data['skin']) / 6000 # with pF
origin = statistics.mean([skin[0], skin[1], skin[2], skin[3], skin[4], skin[5]])
skin = skin - origin
i = 0
while i < np.size(skin):
    if skin[i] < 0:
        skin[i] = 0
    i += 1

force = np.array(data['ft']) / 100  # with N
force  = - force
force  = force - force[0]

# print(skin)
# print(force)

# model = np.polyfit(skin, force, 3)
# print(model[0])
# print(model[1])
# print(model[2])
# end_index = np.size(skin)
plt.plot(skin[18:])
plt.plot(force)
# plt.plot(skin)

plt.show()

