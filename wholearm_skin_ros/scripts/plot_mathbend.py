
import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import mpl_toolkits.axisartist as axisartist

fig = plt.figure(figsize=(4.5, 2.7))
plt.rc('font', family='serif', size='10')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

ddata = pickle.load(open('data_collection_mathbend.pickle', "rb"))
skin = np.array(ddata['skin'])
time = np.array(ddata['skin_time']) - ddata['skin_time'][0]
d = time[173:184]
theta = 358 * d -988.17-4.57-9.15
print(theta)
skin = skin[173:184]/100
skin = skin - skin[0]
print(skin)
m , b = np.polyfit(theta, skin, 1)

fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.plot(theta, m*theta + b, color='#d95f02')
plt.plot(theta, skin, color='#1b9e77')
plt.xlabel(r'Bending Angle $\theta$ ($\degree$)')
plt.ylabel(u'Capacitance (pF)', labelpad = 20)
sns.despine()
plt.legend(['Analytical Calculation', 'Measured Capacitance'], loc = 'lower right', bbox_to_anchor=(0.9, 0), frameon=False)

plt.xlim([25, 99])
plt.ylim([-0.3, 3.7])

ax.xaxis.set_major_locator(plt.NullLocator())
ax.yaxis.set_major_locator(plt.NullLocator())

ax.plot(99, -0.3, marker="4", ms=10, color="k", clip_on=False)
ax.plot(25, 3.7, marker="2", ms=10, color="k", clip_on=False)

desired_ticks = [30, 90]
ax.set_xticks(desired_ticks)

desired_ticks = []
ax.set_yticks(desired_ticks)

ax.plot([30, 30], [-0.3, 0], ls="--", linewidth = 1.5, color='#A6CAF0')
ax.plot([25, 30], [0, 0], ls="--", linewidth = 1.5, color='#A6CAF0')
ax.plot(30, 0, ls="", marker='o', ms=5, color='#A6CAF0')
ax.plot([90, 90], [-0.3, 3.33], ls="--", linewidth = 1.5, color='#92C472')
ax.plot([25, 90], [3.33, 3.33], ls="--", linewidth = 1.5, color='#92C472')
ax.plot(90, 3.33, ls="", marker='o', ms=5, color='#92C472')

ax.text(32.5, -0.7, r'$(\theta_0)$', color='black', fontdict=font_properties)
ax.text(92.5, -0.7, r'$(\theta_1$)', color='black', fontdict=font_properties)

ax.text(21.5, -0.15, r'$C_0$', color='black', fontdict=font_properties)
ax.text(21.5, 3.18, r'$C_1$', color='black', fontdict=font_properties)

plt.tight_layout(pad=0)
plt.savefig("mathbend.pdf", dpi=600)
plt.show()
