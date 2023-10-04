import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import mpl_toolkits.axisartist as axisartist

fig = plt.figure(figsize=(5, 3.5))
plt.rc('font', family = 'serif', size='10')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

# not compressed
data1 = pickle.load(open('data_collection_compress0.pickle', "rb"))
skin1 = np.array(data1['skin'])/1000
skin1 = skin1 * 0.75
s1 = min(skin1)
skin1 = skin1 - s1
force1 = np.array(data1['force'])
force1 = force1 * 0.75
f1 = min(force1)
force1 = force1 - f1
time1 = np.array(data1['skin_time']) - data1['skin_time'][0]

# compressed 10%
data2 = pickle.load(open('data_collection_compress2_1.pickle', "rb"))
skin2 = np.array(data2['skin'])/1000
s2 = min(skin2)
skin2 = skin2 - s2
force2 = np.array(data2['force'])
f2 = min(force2)
force2 = force2 - f2
time2 = np.array(data2['skin_time']) - data2['skin_time'][0]

# compressed 20%
data3 = pickle.load(open('data_collection_compress3_1.pickle', "rb"))
skin3 = np.array(data3['skin'])/1000
skin3 = skin3 * 1.21
s3 = min(skin3)
skin3 = skin3 - s3
force3 = np.array(data3['force']) * 0.8
f3 = min(force3)
force3 = force3 - f3
time3 = np.array(data3['skin_time']) - data3['skin_time'][0]

fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.scatter(skin1/10, force1/10, color = "#1b9e77", s = 1.0)
plt.scatter(skin2/10, force2/10, color = "#d95f02", s = 1.0)
plt.scatter(skin3/10, force3/10, color = "#7570b3", s = 1.0)

plt.xlabel(u'Δ Capacitance (pF)')
plt.ylabel(u'Force (N)')
sns.despine()
plt.legend(['Squeezed 0%', 'Squeezed 10%', 'Squeezed 20%'], loc='upper left', bbox_to_anchor=(0, 1), frameon=False)
xbegin = 0
xend = 0.85
ybegin = 0
yend = 27
plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad=1)

ax.plot(xend, ybegin, ls="", marker="4", ms=10, color="k", clip_on=False)
ax.plot(xbegin, yend, ls="", marker="2", ms=10, color="k", clip_on=False)

plt.show()