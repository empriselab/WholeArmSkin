import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import mpl_toolkits.axisartist as axisartist

fig = plt.figure(figsize=(4.5, 2.7))
plt.rc('font', family='serif', size='10')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

data = pickle.load(open('data_collection_mathpush.pickle', "rb"))
skin = np.array(data['skin'])
time = np.array(data['skin_time']) - data['skin_time'][0]
time = time[119:201] # 82
time = 2.367 - 0.41 * time
skin = skin + 3000
skin = skin[119:201]/1000

m , b = np.polyfit(1/time, skin, 1)

fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.plot(time, skin, color='#1b9e77')
plt.plot(time, m*(1/time) + b, color='#d95f02')
plt.xlabel(u'Thickness of the Taxel (cm)')
plt.ylabel(u'Capacitance (pF)')
sns.despine()
plt.legend(['Analytical Capacitance', 'Measured Capacitance'], loc = 'upper right', bbox_to_anchor=(1, 1), frameon=False)

plt.xlim([0.97, 1.63])
plt.ylim([3.75, 5.8])
ax.xaxis.set_major_locator(plt.NullLocator())
ax.yaxis.set_major_locator(plt.NullLocator())

ax.plot(1.63, 3.75, marker="4", ms=10, color="k", clip_on=False)
ax.plot(0.97, 5.8, marker="2", ms=10, color="k", clip_on=False)

desired_ticks = [1.00, 1.55]
ax.set_xticks(desired_ticks)

ax.plot([1.00, 1.00], [3.75, 5.618], ls="--", linewidth = 1.5, color='#A6CAF0')
ax.plot([0.97, 1.00], [5.618, 5.618], ls="--", linewidth = 1.5, color='#A6CAF0')
ax.plot(1.00, 5.618, ls="", marker='o', ms=5, color='#A6CAF0')
ax.plot([1.55, 1.55], [3.75, 3.850], ls="--", linewidth = 1.5, color='#92C472')
ax.plot([0.97, 1.55], [3.850, 3.850], ls="--", linewidth = 1.5, color='#92C472')
ax.plot(1.55, 3.850, ls="", marker='o', ms=5, color='#92C472')
# desired_ticks = [3.850, 5.618]
desired_ticks = [3.85, 5.62]
ax.set_yticks(desired_ticks)

ax.text(1.03, 3.57, r'$(h_0)$', color='black', fontdict=font_properties)
ax.text(1.58, 3.57, r'$(h_1)$', color='black', fontdict=font_properties)
plt.tight_layout(pad=0)

plt.savefig("mathpush.pdf", dpi=600)
plt.show()
