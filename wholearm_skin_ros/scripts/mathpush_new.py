import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

plt.figure(figsize = (5.0, 3.5))
plt.rc('font', family='serif', size='10')

data = pickle.load(open('data_collection_mathpush.pickle', "rb"))

skin = np.array(data['skin'])
time = np.array(data['skin_time']) - data['skin_time'][0]
time = time[119:201]
time = 2.367 - 0.41 * time
skin = skin + 3000
skin = skin[119:201]/1000

half_sp = 10
whole_sp = 2*half_sp
max_len = len(skin)

t = np.zeros(whole_sp)
c = np.zeros(whole_sp)
print(c)
t = time[0:whole_sp]

i = 1
while i < max_len:
    i = int(i)
    if i+whole_sp > max_len:
        break
    peak_i = i + np.argmax(skin[i:i+whole_sp])
    st = peak_i - half_sp
    en = peak_i + half_sp
    if (st < 0):
        st = 0
    if (en > max_len):
        en = max_len
    i = en +half_sp
    if np.size(c) == 0:
        c = skin[st:en]
    print("st", st, "en", en)
    c = np.vstack((c, skin[st:en]))
    print(c)

c_avg = np.mean(c[1:], axis = 0)
c_stddev = np.std(c[1:], axis = 0)

m , b = np.polyfit(1/t, c_avg, 1)

fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.plot(t, c_avg, color = '#1b9e77')
est = (m+0.02)*(1/t) + b -0.02
plt.plot(t, est, color='#d95f02')
plt.xlabel(u'Thickness of the Taxel (cm)')
plt.ylabel(u'Capacitance (pF)')
sns.despine()
plt.legend(['Analytical Capacitance', 'Measured Capacitance'], loc = 'upper right', bbox_to_anchor=(1, 1), frameon=False)
plt.fill_between(t, c_avg - c_stddev*0.01, c_avg + c_stddev*0.01, alpha = 0.4, color = '#1b9e77')
plt.fill_between(t, 0.999*est, 1.001*est, alpha = 0.4, color = '#d95f02')
plt.show()
