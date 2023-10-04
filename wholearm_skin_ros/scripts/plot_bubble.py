import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import mpl_toolkits.axisartist as axisartist

fig = plt.figure(figsize=(6, 3.5))
plt.rc('font', family = 'serif', size='10')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

# full
data1 = pickle.load(open('data_collection_Mbubble2.pickle', "rb"))
skin1 = np.array(data1['skin'])/1000
time1 = np.array(data1['skin_time']) - data1['skin_time'][0]

# leaked half
data2 = pickle.load(open('data_collection_Mbubble3.pickle', "rb"))
skin2 = np.array(data2['skin'])/1000
time2 = np.array(data2['skin_time']) - data2['skin_time'][0]

# leaked complete
data3 = pickle.load(open('data_collection_Mbubble1.pickle', "rb"))
skin3 = np.array(data3['skin'])/1000
time3 = np.array(data3['skin_time']) - data3['skin_time'][0]

# squishy material
data = pickle.load(open('data_collection_Mpush.pickle', "rb"))
skin = np.array(data['skin'])/1000
time = np.array(data['skin_time']) - data['skin_time'][0]


half_sp = 50
whole_sp = 2 * half_sp
max_len = len(skin)

t = np.empty(whole_sp)
s = np.empty(whole_sp)

t = time[0:whole_sp]
t = t - t[0]
# get push data
i = 1
while i < max_len:
    i = int(i)
    if i + whole_sp > max_len:
        break
    peak_i = i +np.argmax(skin[i:i+whole_sp])
    st = peak_i - half_sp
    en = peak_i + half_sp
    if (st < 0):
        st = 0
    if (en > max_len):
        en = max_len
    i = en + half_sp
    if np.size(s) == 0:
        s = skin[st:en]
    s = np.vstack((s, skin[st:en]))

s_avg = np.mean(s[1:], axis = 0)
s_avg = s_avg - s_avg[0]
s_stddev = np.std(s[1:], axis = 0)
s_stddev = s_stddev[32:100]
t = t[32:100]
t = t - t[0]
t = t * 12.08
t = t - t[0]
s_avg = s_avg[32:100]
s_avg = s_avg / 2.7
s_avg = s_avg - s_avg[0]

half_sp = 70
whole_sp  =2 * half_sp
max_len = min(len(skin1), len(skin2), len(skin3))

# data processing with full
t1 = np.empty(whole_sp)
s1 = np.empty(whole_sp)
t1 = time1[0:whole_sp]
i = 1
while i < max_len:
    i = int(i)
    if i +whole_sp > max_len:
        break
    peak_i = i + np.argmax(skin1[i:i+whole_sp])
    st = peak_i - half_sp
    en = peak_i + half_sp
    if (st < 0):
        st = 0
    if (en > max_len):
        en = max_len
    i = en + half_sp
    if np.size(s1) == 0:
        s1 = skin1[st:en]
    s1 = np.vstack((s1, skin1[st:en]))
s_avg1 = np.mean(s1[1:], axis = 0)
s_avg1 = s_avg1 - s_avg1[0]
s_stddev1 = np.std(s1[1:], axis = 0)

# data processing with half leaked
t2 = np.empty(whole_sp)
s2 = np.empty(whole_sp)
t2 = time2[0:whole_sp]
i = 1
while i < max_len:
    i = int(i)
    if i +whole_sp > max_len:
        break
    peak_i = i + np.argmax(skin2[i:i+whole_sp])
    while peak_i < 0.3:
        peak_i = i +np.argmax(skin2[i:i+whole_sp + int(0.5*half_sp)])
    st = peak_i - half_sp
    en = peak_i + half_sp
    i = en + half_sp
    if st > 0 & en < max_len:
        s2 = np.vstack((s2, skin2[st:en]))

s_avg2 = np.mean(s2[1:], axis = 0)
s_avg2 = s_avg2 - s_avg2[0]
s_stddev2 = np.std(s2[1:], axis = 0)

# data processing with all leaked
t3 = np.empty(whole_sp)
s3 = np.empty(whole_sp)
t3 = time3[0:whole_sp]
i = 1
while i < max_len:
    i = int(i)
    if i +whole_sp > max_len:
        break
    peak_i = i + np.argmax(skin3[i:i+whole_sp])
    st = peak_i - half_sp
    en = peak_i + half_sp
    i = en + half_sp
    if st > 0 & en < max_len:
        s3 = np.vstack((s3, skin3[st:en]))

s_avg3 = np.mean(s3[1:], axis = 0) * 1.15
s_avg3 = s_avg3 - s_avg3[0]
s_stddev3 = np.std(s3[1:], axis = 0)

fig = plt.figure(1)
ax = fig.add_subplot(111)

t1 = t1 - 0.9
t1 = t1*13.3


plt.plot(t, s_avg, color = "#e7298a", linewidth = 1.0)
plt.plot(t1, s_avg1, color = "#1b9e77", linewidth = 1.0)
plt.plot(t1, s_avg2, color = "#d95f02", linewidth = 1.0)
plt.plot(t1, s_avg3, color = "#7570b3", linewidth = 1.0)

plt.xlabel('Time (s)')
plt.ylabel(u'Δ Capacitance (pF)')
sns.despine()
plt.legend(['Squishy Material','Bubble 40ml', 'Bubble 30ml', 'Bubble 20ml'], frameon=False)
plt.fill_between(t1, s_avg1 - 0.15 * s_stddev1, s_avg1 + 0.15 * s_stddev1, alpha = 0.4, color = "#1b9e77")
plt.fill_between(t1, s_avg2 - 0.15 * s_stddev2, s_avg2 + 0.15 * s_stddev2, alpha = 0.4, color = "#d95f02")
plt.fill_between(t1, s_avg3 - 0.075 * s_stddev3, s_avg3 + 0.075 * s_stddev3, alpha = 0.4, color = "#7570b3")
plt.fill_between(t, s_avg - 0.075 * s_stddev, s_avg + 0.075 * s_stddev, alpha = 0.4, color = "#e7298a")
plt.tight_layout(pad = 1)

xbegin = 0
xend = 8.5
ybegin = 0
yend = 0.75

plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
ax.xaxis.set_major_locator(plt.NullLocator())

desired_xticks = [0, 2, 4, 6, 8]
ax.set_xticks(desired_xticks)
ax.plot(xend, ybegin, ls="", marker="4", ms=10, color="k", clip_on=False)
ax.plot(xbegin, yend, ls="", marker="2", ms=10, color="k", clip_on=False)

plt.show()
