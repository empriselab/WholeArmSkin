import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

plt.figure(figsize=(7, 3.5))
plt.rc('font', family='serif', size='18')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 18}

data = pickle.load(open('data_collection_Mpush.pickle', "rb"))


skin = np.array(data['skin'])/1000
time = np.array(data['skin_time']) - data['skin_time'][0]

half_sp = 50
whole_sp = 2 * half_sp
max_len = len(skin)

t = np.empty(half_sp)
s = np.empty(half_sp)

t = time[0:half_sp]

# get push data
i = 1
while i < max_len:
    i = int(i)
    if i + whole_sp > max_len:
        break
    peak_i = i +np.argmax(skin[i:i+whole_sp])
    st = peak_i - half_sp
    en = peak_i
    if (st < 0):
        st = 0
    if (en > max_len):
        en = max_len
    i = en + whole_sp
    if np.size(s) == 0:
        s = skin[st:en]
    s = np.vstack((s, skin[st:en]))

s_avg = np.mean(s[1:], axis = 0) + 6
s_stddev = np.std(s[1:], axis = 0)

fig = plt.figure(1)
ax = fig.add_subplot(111)
t = t[37:48]
d = 3.096 - 2.6 * t
x = 1.5 - d
x = x * 1.6 - 0.012
s_avg = s_avg[37:48]
s_stddev = s_stddev[37:48]

m, b = np.polyfit(1/d, s_avg, 1)
math = m*(1/d)+b
print("m")
print(m)
print("b")
print(b)

# plot
plt.plot(x, math, color = '#d95f02')
plt.plot(x, s_avg, color = '#1b9e77')

plt.xlabel(u'Deformation x (cm)')
plt.ylabel(u'Capacitance (pF)')
sns.despine()
plt.legend(['Analytical Capacitance', 'Measured Capacitance'], loc = 'upper left', bbox_to_anchor=(-0.03,0.97), frameon=False)

plt.fill_between(x, 0.999 * math, 1.001 * math, alpha = 0.4, color = '#d95f02')
plt.fill_between(x, s_avg - 0.3 * s_stddev, s_avg + 0.3 * s_stddev, alpha = 0.4, color = '#1b9e77')
xbegin = -0.017
xend = max(x) + 0.08
ybegin = 6
yend = max(s_avg) + 0.2
plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad = 1)
ax.xaxis.set_major_locator(plt.NullLocator())
ax.yaxis.set_major_locator(plt.NullLocator())
ax.plot(xend, ybegin, ls="", marker="4", ms=12, color="k", clip_on=False, markeredgewidth=1.7)
ax.plot(xbegin, yend, ls="", marker="2", ms=12, color="k", clip_on=False, markeredgewidth=1.7)
print(x)
print(s_avg)
xpoint1 = 0
ypoint1 = 6.11
xpoint2 = 0.69
ypoint2 = 7.64
ax.plot([xpoint1, xpoint1], [ybegin, ypoint1], ls="--", linewidth = 2, color='#92c5de')
ax.plot([xbegin, xpoint1], [ypoint1, ypoint1], ls="--", linewidth = 2, color='#92c5de')
ax.plot(xpoint1, ypoint1, ls="", marker='o', ms=5, color='#92c5de')
ax.plot([xpoint2, xpoint2], [ybegin, ypoint2], ls="--", linewidth = 2, color='#a8ddb5')
ax.plot([xbegin, xpoint2], [ypoint2, ypoint2], ls="--", linewidth = 2, color='#a8ddb5')
ax.plot(xpoint2, ypoint2, ls="", marker='o', ms=5, color='#a8ddb5')
desired_xticks = [xpoint1, xpoint2]
desired_yticks = [ypoint1, ypoint2]
ax.set_xticks(desired_xticks)
ax.set_yticks(desired_yticks)

ax.spines['bottom'].set_linewidth(1.7)  # x轴底部
ax.spines['left'].set_linewidth(1.7)    # y轴左侧
ax.tick_params(width=1.5)

# ax.text(xpoint2 + 0.04, ybegin - 0.2, r'(x)', color = 'black', fontdict=font_properties)

s_stddev = sum(s_stddev)/len(s_stddev)
print(0.3 * s_stddev)

plt.show()