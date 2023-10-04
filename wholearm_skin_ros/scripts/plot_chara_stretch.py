import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

fig = plt.figure(figsize=(5, 3.5))
plt.rc('font', family = 'serif', size='10')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

data1 = pickle.load(open('data_collection_chara_stretch6.pickle', "rb"))
skin1 = np.array(data1['skin'])/1000 * 1.02/2
force1 = np.array(data1['force'])/100 * 0.99 * 10 * 1.16
time1 = np.array(data1['skin_time']) - data1['skin_time'][0]
skin1_1 = np.array(data1['skin'])/1000 * 1.03/2
force1_1 = np.array(data1['force'])/100 * 0.97 * 10 * 1.16
skin1_2 = np.array(data1['skin'])/1000 * 1.05/2
force1_2 = np.array(data1['force'])/100 * 0.98 * 10 * 1.16


data2 = pickle.load(open('data_collection_chara_stretch6.5.pickle', "rb"))
skin2 = np.array(data2['skin'])/1000 * 1.054/2
force2 = np.array(data2['force'])/100 * 1.064* 10 * 1.16
time2 = np.array(data2['skin_time']) - data2['skin_time'][0]
skin2_1 = np.array(data2['skin'])/1000 * 1.03/2
force2_1 = np.array(data2['force'])/100 * 1.07* 10 * 1.16
skin2_2 = np.array(data2['skin'])/1000 * 1.05/2
force2_2 = np.array(data2['force'])/100 * 1.08* 10 * 1.16

data3 = pickle.load(open('data_collection_chara_stretch7.pickle', "rb"))
skin3 = np.array(data3['skin'])/1000 * 1.1
skin3 = skin3 - 1
skin3 = skin3 / 2
force3 = np.array(data3['force'])/100 * 10 * 1.16
time3 = np.array(data3['skin_time']) - data3['skin_time'][0]
skin3_1 = np.array(data3['skin'])/1000 * 1.13
skin3_1 = skin3_1 - 1
skin3_1 = skin3_1 / 2
force3_1 = np.array(data3['force'])/100 * 10 * 1.16
skin3_2 = np.array(data3['skin'])/1000 * 1.15
skin3_2 = skin3_2 - 1
skin3_2 = skin3_2 / 2
force3_2 = np.array(data3['force'])/100 * 10 * 1.16


fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.scatter(skin1, force1, color = "#1b9e77", s = 1)
plt.scatter(skin1_1, force1_1, color = "#1b9e77", s = 1)
plt.scatter(skin1_2, force1_2, color = "#1b9e77", s = 1)

plt.scatter(skin2, force2, color = "#d95f02", s = 1)
plt.scatter(skin2_1, force2_1, color = "#d95f02", s = 1)
plt.scatter(skin2_2, force2_2, color = "#d95f02", s = 1)

plt.scatter(skin3, force3, color = "#7570b3", s = 1)
plt.scatter(skin3_1, force3_1, color = "#7570b3", s = 1)
plt.scatter(skin3_2, force3_2, color = "#7570b3", s = 1)


plt.xlabel(u'Δ Capacitance (pF)')
plt.ylabel(u'Force (N)')
sns.despine()
plt.legend(['Stretched 0%', 'Stretched 8.3%', 'Stretched 16.7%'], loc='upper left', bbox_to_anchor=(0, 1), frameon=False)

xbegin = 0
xend = 4.2
ybegin = 0
yend = 27
plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad=1)
ax.xaxis.set_major_locator(plt.NullLocator())
desired_xticks = [0, 1, 2, 3, 4]
ax.set_xticks(desired_xticks)


plt.show()

