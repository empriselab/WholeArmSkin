import pandas as pd
import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import mpl_toolkits.axisartist as axisartist
from scipy.signal import butter, lfilter

plt.figure(figsize=(5.0, 3.5))
plt.rc('font', family='serif', size='13') 

# define a filter
b1, a1 = butter(N=8, Wn=0.008, btype='low')
b2, a2 = butter(N=8, Wn=0.01, btype='low')

df = pd.read_excel('1.xlsx', sheet_name='Sheet1')
data = df.to_numpy()
data[:10000, 2] = 0
data[12500:,0] = 0

cf = pd.read_excel('2.xlsx', sheet_name='Sheet2')
datac = cf.to_numpy()
datac[4400:,:3] = 0
datac[:2700,2] = 0
datac[:2200,0] = 0

x1 = data[8600:15000, 0] * 0.75
x2 = data[:, 1]
x3 = data[:, 2]

x4 = datac[2000:8000, 0]
x5 = datac[:, 1]
x6 = datac[:, 2]

# filtered data
x1_f = lfilter(b1, a1, x1)
x2_f = lfilter(b1, a1, x2)
x3_f = lfilter(b1, a1, x3)

x4_f = lfilter(b2, a2, x4)
x5_f = lfilter(b2, a2, x5)
x6_f = lfilter(b2, a2, x6)

fig = plt.figure(1)
ax = fig.add_subplot(111)
x1_f[3800:] = 0
x4_f[3800:] = 0

plt.plot(np.linspace(0, 1.5 * (len(x4_f) - 1), len(x4_f)), -x4_f/10 , color = '#1b9e77')
plt.plot(range(len(x1_f)), -x1_f / 2, color = '#d95f02')
# plt.plot(range(len(x2)), x2)
# plt.plot(range(len(x2_f)), -x2_f * 2, color = '#fc4e2a')
# plt.plot(range(len(x3_f)), x3_f * 5, color = '#fed976')

# plt.plot(np.linspace(0, 1.5 * (len(x5_f) - 1), len(x5_f)), -x5_f/10, color = '#41b6c4')
# plt.plot(np.linspace(0, 1.5 * (len(x6_f) - 1), len(x6_f)), -x6_f/10, color = '#a1dab4')

sns.despine()
plt.legend(['CushSense', 'Scuba Fabric'], loc = 'upper left', bbox_to_anchor = (0,1), frameon = False)

plt.xlabel(u'Time (s)')
plt.ylabel(u'Force (N)')

# plt.ylim([-10, 25])
xbegin = 0
xend = 4700
ybegin = 0
yend = 18

plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad = 1)

ax.xaxis.set_major_locator(plt.NullLocator())
desired_xticks = [0, 1000, 2000, 3000, 4000]
ax.set_xticks(desired_xticks)

ax.spines['bottom'].set_linewidth(1.5)
ax.spines['left'].set_linewidth(1.5)

ax.yaxis.set_major_locator(plt.NullLocator())
desired_yticks = [0, 4, 8, 12, 16]
ax.set_yticks(desired_yticks)

plt.show()




