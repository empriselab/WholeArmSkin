import matplotlib.pyplot as plt
import numpy as np
import pickle
import seaborn as sns

import matplotlib.animation as animation

plt.rc('font', family='serif', size='15')
plt.rcParams['font.weight'] = 'bold'

data = pickle.load(open('data_collection_video_making.pickle', "rb"))

skin = np.array(data['skin']) / 1000
time = np.array(data['skin_time']) - data['skin_time'][0]

skin = skin[210:532]
time = time[210:532]
time = time - time[0]

for i in range(len(time)):
    if skin[i] < 0:
        skin[i] = 0
    if time[i] < 2:
        skin[i] = skin[i] * 1.072


fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot(time, skin, color='#1b9e77', linewidth = 6)


def init():
    ax.set_xlim(0, max(time))
    ax.set_ylim(0, max(skin) + 0.5)
    return line,

def animate(i):
    x = time[:i] 
    y = skin[:i]
    line.set_data(x, y)
    return line,

ani = animation.FuncAnimation(fig, animate, interval = 20, frames=len(time), init_func=init, blit=True)
# ani = animation.FuncAnimation(fig, animate, interval = 20, init_func=init, blit=True)

# ax.set_ylim(0, 4.5)
sns.despine()
ax.xaxis.set_major_locator(plt.NullLocator())
ax.yaxis.set_major_locator(plt.NullLocator())
desired_xticks = [0, 1, 2, 3, 4, 5]
desired_yticks = [0, 1, 2, 3, 4]
ax.set_xticks(desired_xticks)
ax.tick_params(axis='x', width=2)
ax.set_yticks(desired_yticks)
ax.tick_params(axis='y', width=2)
ax.spines['bottom'].set_linewidth(2)
ax.spines['left'].set_linewidth(2)

xbegin = 0
ybegin = 0
xend = max(time)
yend = max(skin) + 0.5

# ax.plot(xend, ybegin, ls="", marker="4", ms=18, color="k", clip_on=False)
# ax.plot(xbegin, yend, ls="", marker="2", ms=18, color="k", clip_on=False)
plt.xlabel(u'Time (s)')
plt.ylabel(u'Capacitance (pF)')
plt.tight_layout(pad =0)
plt.legend()


plt.show()