import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

plt.figure(figsize=(20.0, 3.0))
plt.rc('font', family='serif', size='10') 

fig = plt.figure(1)
ax = fig.add_subplot(111)

data1 = pickle.load(open('data_collection_100HZ.pickle', "rb"))
data2 = pickle.load(open('data_collection_200HZ.pickle', "rb"))
data4 = pickle.load(open('data_collection_400HZ.pickle', "rb"))

skin1 = np.array(data1['skin'])/1000
time1 = np.array(data1['skin_time']) - data1['skin_time'][0]
time1 = time1* 4.57

skin2 = np.array(data2['skin'])/1000
time2 = np.array(data2['skin_time']) - data2['skin_time'][0]
time2 = time2 * 4.57
len = len(time2)
print("len")
skin2 = skin2[1016: 2220]
skin2 = skin2 * 22
time2 = time2[1016:2220]
plt.plot(time2, skin2)
sns.despine()
plt.tight_layout(pad = 1)

# print(len)


# plt.plot(time2, skin2)


# skin4 = np.array(data4['skin'])/1000
# time4 = np.array(data4['skin_time']) - data4['skin_time'][0]
# time4 = time4 * 4.57

# half_sp = 55
# whole_sp = 2 * half_sp
# max_len = len(skin1)

# t = np.empty(whole_sp)
# s1 = np.empty(whole_sp)
# s2 = np.empty(whole_sp)
# s4 = np.empty(whole_sp)

# t = time1[0:whole_sp]

# # get 100HZ data
# i = 1
# while i < max_len:
#     i = int(i)
#     if i + whole_sp > max_len:
#         break
#     peak_i = i +np.argmax(skin1[i:i+whole_sp])
#     st = peak_i - half_sp
#     en = peak_i + half_sp
#     if (st < 0):
#         st = 0
#     if (en > max_len):
#         en = max_len
#     i = en + half_sp
#     if np.size(s1) == 0:
#         s1 = skin1[st:en]
#     s1 = np.vstack((s1, skin1[st:en]))
    

# # get 200HZ data
# i = 1
# while i < max_len:
#     i = int(i)
#     if i +whole_sp > max_len:
#         break
#     peak_i = i +np.argmax(skin2[i:i+whole_sp])
#     st = peak_i - half_sp
#     en = peak_i + half_sp
#     if(st < 0):
#         st = 0
#     if(en > max_len):
#         en = max_len
#     i = en + half_sp
#     if np.size(s2) == 0:
#         s2 = skin2[st:en]
#     s2 = np.vstack((s2, skin2[st:en]))

# # get 400HZ data
# i = 1
# while i <max_len:
#     i = int(i)
#     if i +whole_sp > max_len:
#         break
#     peak_i = i +np.argmax(skin4[i:i+whole_sp])
#     st = peak_i - half_sp
#     en = peak_i + half_sp
#     if(st < 0):
#         st = 0
#     if(en > max_len):
#         en = max_len
#     i = en + half_sp
#     if np.size(s4) == 0:
#         s4 = skin4[st:en]
#     s4 = np.vstack((s4, skin4[st:en]))

# s1_avg = np.mean(s1[1:], axis = 0)
# s1_stddev = np.std(s1[1:], axis = 0)
# s2_avg = np.mean(s2[1:], axis = 0)
# s2_stddev = np.std(s2[1:], axis = 0)
# s4_avg = np.mean(s4[1:], axis = 0)
# s4_stddev = np.std(s4[1:], axis = 0)

# s1_avg = s1_avg * 0.125 / 1.578
# s2_avg = s2_avg * 0.172 / 1.758
# s4_avg = s4_avg * 0.060 / 1.741

# t = t[34:80]
# t = t - t[0]
# t = t * 2
# s1_avg = s1_avg[34:80] * 10
# s2_avg = s2_avg[34:80] * 10
# s4_avg = s4_avg[34:80] * 10


# plt.plot(t, s2_avg, color = '#d95f02')
# plt.plot(t, s1_avg, color = '#1b9e77')
# plt.plot(t, s4_avg, color = '#7570b3')
# print(max(s1_avg))
# print(max(s2_avg))
# print(max(s4_avg))


# plt.xlabel(u'Time (s)')
# plt.ylabel(u'Δ Capacitance (pF)')
# sns.despine()
# plt.legend(['2 x 2', '4 x 4', '8 x 8'], loc='upper right', bbox_to_anchor=(1, 1), frameon=False)
# print(len(t))
# plt.fill_between(t, s1_avg - 0.03, s1_avg + 0.03, alpha = 0.5, color = '#1b9e77')
# plt.fill_between(t, s2_avg - 0.04, s2_avg + 0.04, alpha = 0.5, color = '#d95f02')
# plt.fill_between(t, s4_avg - 0.01, s4_avg + 0.01, alpha = 0.5, color = '#7570b3')

# plt.xlim([0, 197])
# plt.ylim([0, 45])
# ax.plot((1), (0), ls="", marker="4", ms=10, color="k", clip_on=False)
# ax.plot((0), (1), ls="", marker="2", ms=10, color="k",
#     transform=ax.get_xaxis_transform(), clip_on=False)

# plt.tight_layout(pad = 1)

# ax.xaxis.set_major_locator(plt.NullLocator())
# desired_xticks = [0,2,4,6]
# ax.set_xticks(desired_xticks)

# ax.yaxis.set_major_locator(plt.NullLocator())
# desired_yticks = [0, 0.5, 1, 1.5, 2]
# ax.set_yticks(desired_yticks)

# ax.plot(7.3, 0, ls="", marker="4", ms=10, color="k", clip_on=False)
# ax.plot((0), (1), ls="", marker="2", ms=10, color="k",
#     transform=ax.get_xaxis_transform(), clip_on=False)
# plt.savefig("freq.pdf", dpi=600)

plt.show()
