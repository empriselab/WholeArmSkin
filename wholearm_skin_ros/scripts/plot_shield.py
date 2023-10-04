import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import mpl_toolkits.axisartist as axisartist

fig = plt.figure(figsize=(5, 3.5))
plt.rc('font', family='serif', size='10')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

tdata = pickle.load(open('data_collection_ontable_noshield.pickle', "rb"))
tskin = np.array(tdata['skin'])
ttime = np.array(tdata['skin_time']) - tdata['skin_time'][0]
ttime = ttime[138 : 688]
ttime = ttime - ttime[0]
tsum = 0
for i in range(138, 687):
    tsum += tskin[i]
tskin = tskin[138: 688] - tsum/550
tskin = tskin/1000

adata = pickle.load(open('data_collection_onarm_noshield.pickle', "rb"))
askin = np.array(adata['skin'])
atime = np.array(adata['skin_time']) - adata['skin_time'][0]
print(len(atime))
atime = atime[208: 758]
atime = atime - atime[0]
asum = 0
for i in range(208, 758):
    asum += askin[i]
askin = askin[208: 758] - asum/550
askin = askin/1000

sdata = pickle.load(open('data_collection_onarm_shield.pickle', "rb"))
sskin = np.array(sdata['skin'])
stime = np.array(sdata['skin_time']) - sdata['skin_time'][0]
print(len(stime))
stime = stime[138: 688]
stime = stime - stime[0]
ssum = 0
for i in range(138, 687):
    ssum += sskin[i]
sskin = sskin[138: 688] - ssum/550
sskin = sskin/1000

fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.plot(stime, sskin, color='#1b9e77') # onarm shield / green
plt.plot(atime, askin, color='#d95f02') # onarm noshield / orange
plt.plot(ttime, tskin, color='#7570b3') # ontable / purple
plt.xlabel(u'Time (s)')
plt.ylabel(u'Capacitance (pF)')
sns.despine()
plt.legend(['On Arm with Shielding', 'On Arm without Shielding', 'On Table with Shielding'], loc = 'upper right', bbox_to_anchor=(1, 1), frameon=False)

xbegin = 0
xend = 9.2
xaxisend = 9.5
ybegin = -0.07
yend = 0.10

ax.yaxis.set_major_locator(plt.NullLocator())

apeak = max(askin) # onarm without shielding
abottom = min(askin)
speak = max(sskin) # onarm with shielding
sbottom = min(sskin)
tpeak = max(tskin) # ontable
tbottom = min(tskin)
print(tpeak)
print(tbottom)
# on table without shielding
ax.plot([xbegin, xend], [apeak, apeak], ls="--", linewidth = 1.5, color='#fdc086')
ax.plot([xbegin, xend], [abottom, abottom], ls="--", linewidth = 1.5, color='#fdc086')
# on arm with shielding
ax.plot([xbegin, xend], [speak, speak], ls="--", linewidth = 1.5, color='#7fc97f')
ax.plot([xbegin, xend], [sbottom, sbottom], ls="--", linewidth = 1.5, color='#7fc97f')
# on table
ax.plot([xbegin, xend], [tpeak, tpeak], ls="--", linewidth = 1, color='#beaed4')
ax.plot([xbegin, xend], [tbottom, tbottom], ls="--", linewidth = 1, color='#beaed4')


desired_yticks = [apeak, abottom, speak, sbottom]
plt.yticks(desired_yticks, [f'{val:.2f}' for val in desired_yticks])
ax.set_yticks(desired_yticks)

plt.xlim([xbegin, xaxisend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad = 1)

ax.plot(xaxisend, ybegin, ls="", marker="4", ms=10, color="k", clip_on=False)
ax.plot(xbegin, yend, ls="", marker="2", ms=10, color="k", clip_on=False)

plt.show()
