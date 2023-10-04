import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

plt.figure(figsize=(5.0, 3.5))
plt.rc('font', family='serif', size='10') 

passive_data = pickle.load(open('data_collection_passive_shielding.pickle', "rb"))
active_data = pickle.load(open('data_collection_active_shielding.pickle', "rb"))
no_data = pickle.load(open('data_collection_no_shielding.pickle', "rb"))

pskin = np.array(passive_data['skin'])/1000
ptime = np.array(passive_data['skin_time']) - passive_data['skin_time'][0]

askin = np.array(active_data['skin'])/1000
atime = np.array(active_data['skin_time']) - active_data['skin_time'][0]

nskin = np.array(no_data['skin'])/1000
ntime = np.array(no_data['skin_time']) - no_data['skin_time'][0]

t = min(len(ptime), len(atime), len(ntime))
ptime = ptime[0:t]
pskin = pskin[0:t]
p_avg = sum(pskin)/len(pskin)
pskin = pskin - p_avg
askin = askin[0:t]
a_avg = sum(askin)/len(askin)
nskin = nskin[0:t]
n_avg = sum(nskin)/len(nskin)

fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.plot(ptime, pskin, color = '#1b9e77')
plt.plot(ptime, askin, color = '#d95f02')
plt.plot(ptime, nskin, color = '#7570b3')
print(max(pskin))
print(min(pskin))
print(max(askin))
print(min(askin))
print(max(nskin))
print(min(nskin))

plt.xlabel('Time (s)')
plt.ylabel(u'Δ Capacitance (pF)')
sns.despine()
# plt.legend(['passive shielding', 'active shielding', 'no shileding'], loc = 'upper left', bbox_to_anchor = (0,1.05), frameon = False)

xbegin = 0
xend = 28
ybegin = -0.5
yend = 0.7

plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad = 1)
ax.yaxis.set_major_locator(plt.NullLocator())
desired_yticks = [-0.4, -0.2, 0.0, 0.2, 0.4, 0.6]
ax.set_yticks(desired_yticks)

ax.plot(xend, ybegin, ls="", marker="4", ms=13, color="k", clip_on=False)
ax.plot(xbegin, yend, ls="", marker="2", ms=13, color="k", clip_on=False)

# ax.spines['bottom'].set_linewidth(1.7) 
# ax.spines['left'].set_linewidth(1.7)   
# ax.tick_params(width=1.5)

plt.show()
