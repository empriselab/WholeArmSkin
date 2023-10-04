import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import mpl_toolkits.axisartist as axisartist

fig = plt.figure(figsize=(7, 3.5))
plt.rc('font', family = 'serif', size='17')
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 17}

data = pickle.load(open('data_collection_Mstretch.pickle', "rb"))
skin = np.array(data['skin'])/1000
time = np.array(data['skin_time']) - data['skin_time'][0]

fig = plt.figure(1)
ax = fig.add_subplot(111)

half_sp = 65
whole_sp = 2*half_sp
max_len = len(skin)

t = np.empty(half_sp)
s = np.empty(half_sp)

t = time[0:half_sp]

# get compression data
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
    # plt.plot(t,skin[st:en], '#1b9e77')

s_avg = np.mean(s[1:], axis = 0)
s_stddev = np.std(s[1:], axis = 0)
t = t[53:65]
p = 89.67 * t - 79.18 - 0.026
s_avg = s_avg[53:65] - s_avg[53] + 6
s_stddev = s_stddev[53:65]

p = p / 100

coff = np.polyfit(p, s_avg, 2)
a = coff[0]
b = coff[1]
c = coff[2]
print(a)
print(b)
print(c)

math = a*p*p + b*p + c
plt.plot(p, math, color = '#d95f02')
plt.plot(p, s_avg, color = '#1b9e77')

plt.xlabel(r'Degree of Compression $\alpha$')
plt.ylabel(u'Capacitance (pF)')
sns.despine()
plt.legend(['Analytical Capacitance', 'Measured Capacitance'], loc = 'upper left', bbox_to_anchor = (-0.06, 0.97), frameon = False)

plt.fill_between(p, 0.999 * math, 1.001 * math, alpha = 0.5, color = '#d95f02')
plt.fill_between(p, s_avg - 0.2 * s_stddev, s_avg + 0.2*s_stddev, alpha = 0.5, color = '#1b9e77')

xbegin = -0.008
xend = 0.185
ybegin = 5.97
yend = 6.16
plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad = 1)
ax.xaxis.set_major_locator(plt.NullLocator())
ax.yaxis.set_major_locator(plt.NullLocator())
ax.plot(xend, ybegin, ls="", marker="4", ms=12, color="k", clip_on=False, markeredgewidth=1.7)
ax.plot(xbegin, yend, ls="", marker="2", ms=12, color="k", clip_on=False, markeredgewidth=1.7)
print(p)
print(s_avg)
xpoint1 = 0
ypoint1 = 6
xpoint2 = 0.16
ypoint2 = 6.14
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

# ax.text(xpoint2 + 0.009, ybegin - 0.020, r'$(\alpha)$', color = 'black', fontdict=font_properties)


s_stddev = sum(s_stddev)/len(s_stddev)

print(0.2 * s_stddev)

plt.show()