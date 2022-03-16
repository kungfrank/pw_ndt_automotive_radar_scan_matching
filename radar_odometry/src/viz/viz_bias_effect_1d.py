import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

plt.style.use('seaborn-deep')
'''
x = np.random.normal(0, 2, 5000)
y = np.random.normal(0, 2, 2000)
bins = np.linspace(-10, 10, 30)

#plt.hist([x, y], bins, label=['x', 'y'])
plt.hist(x, bins, alpha=0.5, label='x')
plt.hist(y, bins, alpha=0.5, label='y')

plt.legend(loc='upper right')
'''

half_grid_size = 100
slope = 0.5


origin = []
unbias1 = []
unbias2 = []

for i in range(half_grid_size):
  for j in range(int(i*slope)):
    unbias1.append(i)
for i in range(half_grid_size):
  for j in range(int((half_grid_size-i)*slope)):
    unbias1.append(i+half_grid_size)

for i in range(half_grid_size):
  for j in range(int(i*slope + 30)):
    unbias2.append(i)
for i in range(half_grid_size):
  for j in range(int((half_grid_size-i)*slope + 30)):
    unbias2.append(i+half_grid_size)

for i in range(half_grid_size):
  for j in range(int(i*slope + 205)):
    origin.append(i)
for i in range(half_grid_size):
  for j in range(int((half_grid_size-i)*slope + 205)):
    origin.append(i+half_grid_size)

#print(x)
plot1 = plt.figure(1)
plt.hist(origin, bins=half_grid_size*2, alpha=1, label='r', color='red')
plt.hist(unbias2, bins=half_grid_size*2, alpha=1, label='g', color='g')
plt.hist(unbias1, bins=half_grid_size*2-3, alpha=1, label='b', color='blue')
#plt.show()

unbias1_var = np.var(unbias1)
unbias1_mean = np.mean(unbias1)
print(unbias1_mean,math.sqrt(unbias1_var))

unbias2_var = np.var(unbias2)
unbias2_mean = np.mean(unbias2)
print(unbias2_mean,math.sqrt(unbias2_var))

origin_var = np.var(origin)
origin_mean = np.mean(origin)
print(origin_mean,math.sqrt(origin_var))

def gaussian(pos, mu, Sigma):
    N = np.sqrt((2*np.pi)) * Sigma
    fac = ((pos-mu) / Sigma)**2
    return np.exp(-fac / 2) / N

x = np.linspace(0, half_grid_size*2, half_grid_size*2)
origin_y = gaussian(x, origin_mean, math.sqrt(origin_var))
unbias1_y = gaussian(x, unbias1_mean, math.sqrt(unbias1_var))
unbias2_y = gaussian(x, unbias2_mean, math.sqrt(unbias2_var))

plot2 = plt.figure(2)
plt.plot(x, origin_y, '-', color='red')
plt.plot(x, unbias1_y, '-', color='blue')
plt.plot(x, unbias2_y, '-', color='g')
plt.show()
