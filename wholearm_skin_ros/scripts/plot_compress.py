import pickle
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

plt.figure(figsize=(7, 3.5))
plt.rc('font', family='serif', size='10') 
font_properties = {'family': 'serif', 'weight': 'normal', 'size': 10}

data = pickle.load(open('data_collection_compress0.pickle', "rb"))

skin = np.array(data['skin'])/1000
print(skin[0:10])
time = np.array(data['skin_time']) - data['skin_time'][0]