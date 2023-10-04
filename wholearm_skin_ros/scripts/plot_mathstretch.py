
import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

fig = plt.figure(figsize=(4.5, 2.7))
plt.rc('font', family='serif', size='10')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

ddata = pickle.load(open('data_collection_mathstretch.pickle', "rb"))
skin = np.array(ddata['skin'])
time = np.array(ddata['skin_time']) - ddata['skin_time'][0]
d = time[93:110]
d = 0.6 * d + 0.099
d = 100 * d
d = d -d[0]
skin = skin[93:110]/1000
skin = skin - skin[0]
print(skin)
m , b = np.polyfit(d, skin, 1)

fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.plot(d, m*d + b, color='#d95f02')
plt.plot(d, skin, color='#1b9e77')
plt.xlabel(r'Strech Percentage $\alpha$ (%)')
plt.ylabel(u'Capacitance (pF)', labelpad= 15)
plt.tight_layout(pad=0)
sns.despine()
plt.legend(['Analytical Calculation', 'Measured Capacitance'], loc = 'lower right', bbox_to_anchor=(0.95, 0), frameon=False)

plt.xlim([-1, 17])
plt.ylim([-0.3, 2.7])

ax.xaxis.set_major_locator(plt.NullLocator())
ax.yaxis.set_major_locator(plt.NullLocator())

ax.plot(17, -0.3, marker="4", ms=10, color="k", clip_on=False)
ax.plot(-1, 2.7, marker="2", ms=10, color="k", clip_on=False)

desired_ticks = [0, 16]
ax.set_xticks(desired_ticks)

desired_ticks = []
ax.set_yticks(desired_ticks)

ax.plot([0, 0], [-0.3, 0], ls="--", linewidth = 1.5, color='#A6CAF0')
ax.plot([-1, 0], [0, 0], ls="--", linewidth = 1.5, color='#A6CAF0')
ax.plot(0, 0, ls="", marker='o', ms=5, color='#A6CAF0')
ax.plot([16, 16], [-0.3, 2.437], ls="--", linewidth = 1.5, color='#92C472')
ax.plot([-1, 16], [2.437, 2.437], ls="--", linewidth = 1.5, color='#92C472')
ax.plot(16, 2.437, ls="", marker='o', ms=5, color='#92C472')

ax.text(-1.8, -0.02, r'$C_0$', color='black', fontdict=font_properties)
ax.text(-1.8, 2.417, r'$C_1$', color='black', fontdict=font_properties)

plt.tight_layout(pad=0)
plt.savefig("mathstretch.pdf", dpi=600)
plt.show()
