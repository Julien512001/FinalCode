import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


df = pd.read_csv('data/position.txt', delimiter=',')
#df1 = pd.read_csv("Position/odometry.txt", delimiter=',')


# Extraction des valeurs des deux colonnes dans des tableaux NumPy
x = df.iloc[:, 0].values.astype(float)
y = df.iloc[:, 1].values.astype(float)
theta = df.iloc[:, 2].values.astype(float)
""" 
x_odo = df1.iloc[:, 0].values.astype(float)
y_odo = df1.iloc[:, 1].values.astype(float)
theta_odo = df1.iloc[:, 2].values.astype(float)
 """


rad = np.radians(theta)
#rad_odo = np.radians(theta_odo)




plt.scatter(x,y)
#plt.scatter(x_odo, y_odo)

plt.quiver(x, y, np.cos(rad), np.sin(rad), angles='xy', scale_units='xy', scale=100, color='red', label='Orientation')

#plt.xlim([-2.0, 0.0])
#plt.ylim([0.0, 3.0])
plt.xlim([0.0, 2.0])
plt.ylim([0.0, 3.0])
plt.gca().set_aspect(1)
plt.show()