import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

plt.figure(figsize=(7, 3.5))
plt.rc('font', family='serif', size='10')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

data = pickle.load(open('data_collection_mathmodel_value.pickle', "rb"))

skin = np.array(data['skin'])
force = np.array(data['force'])
time = np.array(data['skin_time']) - data['skin_time'][0]

print(len(skin))
time = time[451:540]
time = time - time[0]

print(time)
h = np.zeros(len(time))
math = np.zeros(len(time))

for i in range(len(time)):
    if time[i] < 0.15:
        h[i] = 1.6
        math[i] = 5.88*(1/1.6) + 2.16
    elif time[i] < 0.783:
        h[i] = 1.6948 - 0.6319 * time[i]
        math[i] = 5.88 * (1/h[i]) + 2.16
    elif time[i] < 1.3:
        h[i] = 0.5942 + 0.7737 * time[i]
        math[i] = 5.88 * (1/h[i]) + 2.16
    else:
        h[i] = 1.6
        math[i] = 5.88*(1/1.6) + 2.16

skin = skin[451:540]
force = force[451:540]

# skin = skin * max(force)/max(skin)
# math = math * max(force)/max(math)

a3 = 0.7790287413076263
a2 = -0.003307914541061115
a1 = -3.9204118247392246e-07
a0 = -2.7261725080037385e-10

skin = skin/(1000 * 11.93) *1.26

calibration_force = a3 + a2 * (skin*1000) + a1* (skin*1000)**2 + a0 * (skin*1000)**3
calibration_force = calibration_force - calibration_force[0]
# skin = skin + 5.831
math  = math - 5.831
calculated_force = a3 + a2 * (math*1000) + a1* (math*1000)**2 + a0 * (math*1000)**3
calculated_force = calculated_force - calculated_force[0]

actual_force = force / 286 * 5.4
actual_force = actual_force - actual_force[0]

fig = plt.figure(1)
ax = fig.add_subplot(111)

time = time * 2
plt.plot(time, -calibration_force, color = '#d95f02')
plt.plot(time, -calculated_force, color = '#1b9e77')
plt.plot(time, actual_force, color = '#7570b3')
# plt.plot(time, force, color = 'blue')

plt.xlabel(u'Time (s)')
plt.ylabel(u'Force (N)')
sns.despine()

plt.legend(['Force calculated with measured capacitance', 'Force calculated with math model', 'Force measured by load cell sensor'], loc = 'upper left', bbox_to_anchor=(0,1), frameon=False)

xbegin = 0
xend = 2.9
ybegin = 0
yend = 6.5
plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad = 1)
ax.xaxis.set_major_locator(plt.NullLocator())
ax.yaxis.set_major_locator(plt.NullLocator())
ax.plot(xend, ybegin, ls="", marker="4", ms=12, color="k", clip_on=False)
ax.plot(xbegin, yend, ls="", marker="2", ms=12, color="k", clip_on=False)

desired_xticks = [0, 1, 2]
desired_yticks = [0, 2, 4, 6]
ax.set_xticks(desired_xticks)
ax.set_yticks(desired_yticks)

plt.show()