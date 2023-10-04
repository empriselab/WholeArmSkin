import pandas as pd
import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import mpl_toolkits.axisartist as axisartist
from scipy.signal import butter, lfilter

# plt.figure(figsize=(5.0, 3.5))
# plt.rc('font', family='serif', size='13') 

# define a filter
b1, a1 = butter(N=8, Wn=0.01, btype='low')
b2, a2 = butter(N=8, Wn=0.01, btype='low')

sheet1 = pd.read_excel('3.xlsx', sheet_name='sheet1')
sheet2 = pd.read_excel('3.xlsx', sheet_name='sheet2')
sheet3 = pd.read_excel('3.xlsx', sheet_name='sheet3')
sheet4 = pd.read_excel('3.xlsx', sheet_name='sheet4')
sheet5 = pd.read_excel('3.xlsx', sheet_name='sheet5')
data1 = sheet1.to_numpy()
data2 = sheet2.to_numpy()
data3 = sheet3.to_numpy()
data4 = sheet4.to_numpy()
data5 = sheet5.to_numpy()

# data1 CushSense
c1_34 = data1[9400+1000-500:9400+3090+1000-500-500,33] # 412
c1_34 = abs(c1_34)
c1_34 = c1_34 / max(c1_34) *10
c1_34[:500] = 0
c1_35 = data1[9400+1000:9400+3090+1000-500,34] # 380
c1_35 = abs(c1_35)
c1_35 = c1_35 / max(c1_35) *10

# data3
c3_34 = data3[:,33]
c3_34 = abs(c3_34)
c3_34 = c3_34 / max(c3_34) *10
c3_35 = data3[:,34]
c3_35 = abs(c3_35)
c3_35 = c3_35 / max(c3_35) *10
c3_33 = data3[:,32]
c3_33 = abs(c3_33)
c3_33 = c3_33 / max(c3_33) *10

plt.plot(range(len(c3_34)), c3_34, color = 'black')
plt.plot(range(len(c3_35)), c3_35, color = 'green')

# plt.plot(range(len(data)), taxel1_f, color = '#1b9e77')
# plt.plot(range(len(taxel2_f)), taxel2_f, color = '#d95f02')

sns.despine()

plt.show()




