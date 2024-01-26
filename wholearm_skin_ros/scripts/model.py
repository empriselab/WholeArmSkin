#!/usr/bin/env python3

import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns 
import statistics

data = pickle.load(open('data_collection_link1_2.pickle', "rb"))

# get skin and force data
skin = np.array(data['skin'])
force = np.array(data['ft'])
time = np.array(data['skin_time']) - data['skin_time'][0]

# pre-process skin and force data
origin = statistics.mean([skin[0], skin[1], skin[2], skin[3], skin[4], skin[5]])
skin = skin - origin
i = 0
while i < np.size(skin):
    if skin[i] < 0:
        skin[i] = 0
    i += 1
force  = - force
force  = force - force[0]
skin = skin[135:]
force = force[135-22:len(force)-22]


# define some variables
half_sp = 35
whole_sp = 2*half_sp
max_len = len(skin)

time_re = np.empty(whole_sp)
skin_re = np.empty(whole_sp)
force_re = np.empty(whole_sp)

time_re = time[0:whole_sp]


# slice the data for each press
i = 1
j = 1
while i < max_len:
    i = int(i)
    if i+whole_sp > max_len:
        break
    peak_i = i + np.argmax(skin[i:i+whole_sp])
    st = peak_i - half_sp
    en = peak_i + half_sp
    if (st < 0):
        skin_re = skin[0:en]
        force_re = force[0:en]
        add = [0]* (-st)
        skin_add = np.concatenate((add, skin_re), axis=0)
        force_add = np.concatenate((add, force_re), axis=0)
    elif (en > max_len):
        skin_re = skin[st:max_len]
        force_re = force[st:max_len]
        add = [0]*(en - max_len)
        skin_add = np.concatenate((skin_re, add), axis=0)
        force_add = np.concatenate((force_re, add), axis=0)
    else:
        skin_add = skin[st:en]
        force_add = force[st:en]
    
    if j == 1:
        skin_re = skin_add
        force_re = force_add

    i = en + half_sp
    j += 1
    print(j)
    if not (j == 7):
        skin_re = np.vstack((skin_re, skin_add))
        force_re = np.vstack((force_re, force_add))
        plt.plot(time_re, skin_add/1000, 'tomato')
        plt.plot(time_re, force_add, 'blue')

# get average for each time point
skin_avg = np.mean(skin_re[1:], axis=0)
force_avg = np.mean(force_re[1:], axis=0)
# plt.plot(time_re, skin_avg, 'red')
plt.plot(time_re, force_avg, 'blue')

# c_stddev = np.std(c_re[1:], axis=0) # get stddev for each time point
# nc_stddev = np.std(nc_re[1:], axis=0) # get stddev for each time point

# calculate the model
a0, a1, a2, a3, a4 = np.polyfit(skin, force, 4)
cali_skin = a0 * skin_avg**4 + \
            a1 * skin_avg**3 + \
            a2 * skin_avg**2 + \
            a3 * skin_avg**1 + a4
print(a0,a1,a2,a3,a4)
plt.plot(time_re, cali_skin, 'green')
plt.show()