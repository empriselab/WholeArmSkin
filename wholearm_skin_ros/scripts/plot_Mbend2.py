import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

plt.figure(figsize=(7, 3.5))
plt.rc('font', family='serif', size='17') 
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 17}

data = pickle.load(open('data_collection_Mbend2.pickle', "rb"))

skin = np.array(data['skin'])/1000
time = np.array(data['skin_time']) - data['skin_time'][0]

half_sp = 80
whole_sp = 2 * half_sp
max_len =  len(skin)

t = np.empty(half_sp)
s = np.empty(half_sp)

t = time[0:half_sp]

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

s_avg = np.mean(s[1:], axis = 0)
s_stddev = np.std(s[1:], axis = 0)

t = t[66:80]
s_avg = (s_avg[66:80] - s_avg[66]) * 1.5 + 6
s_stddev = s_stddev[66:80]
angle = 414.75*t - 456.22
fig = plt.figure(1)
ax = fig.add_subplot(111)

angle = angle*3.1415926 / 180

coff = np.polyfit(angle, s_avg, 2)
a = coff[0]
b = coff[1]
c = coff[2]
math = a * angle * angle + b * angle + c


plt.plot(angle, math, color = '#d95f02')
plt.plot(angle, s_avg, color = '#1b9e77')

plt.xlabel(r'Bending Angle $\theta$ (rad)')
plt.ylabel(u'Capacitance (pF)')
sns.despine()
plt.legend(['Analytical Capacitance', 'Measured Capacitance'], loc = 'upper left', bbox_to_anchor = (-0.03,0.98), frameon = False)
plt.fill_between(angle, 0.999 * math, 1.001 * math, alpha = 0.5, color = '#d95f02')
plt.fill_between(angle, s_avg - 0.5 * s_stddev, s_avg + 0.5 * s_stddev, alpha = 0.5, color = '#1b9e77')

xbegin = -5 * 3.1415926 / 180
xend = 102*3.1415926 / 180
ybegin = 5.98
yend = 6.12
plt.xlim([xbegin, xend])
plt.ylim([ybegin, yend])
plt.tight_layout(pad = 1)
ax.xaxis.set_major_locator(plt.NullLocator())
ax.yaxis.set_major_locator(plt.NullLocator())
ax.plot(xend, ybegin, ls="", marker="4", ms=12, color="k", clip_on=False, markeredgewidth=1.7)
ax.plot(xbegin, yend, ls="", marker="2", ms=12, color="k", clip_on=False, markeredgewidth=1.7)
print(angle)
print(s_avg)
xpoint1 = 0.00
ypoint1 = 6.00
xpoint2 = 3.1415926/2
ypoint2 = 6.11
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

# ax.text(xpoint2 + 4.5*3.1415926/180, ybegin - 0.015, r'$(\theta)$', color = 'black', fontdict=font_properties)

print(a)
print(b)
print(c)

s_stddev = sum(s_stddev) / len(s_stddev)
print(0.5 * s_stddev)

plt.show()
