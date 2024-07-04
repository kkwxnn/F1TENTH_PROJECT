
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# data = pd.read_csv('/home/tanakon/FRA532EXAM_31_62_WS/university_records.csv')
data = pd.read_csv('/home/tuchapong1234/FRA532EXAM_WS/src/circle.csv')


fig, ax = plt.subplots()

x1 = np.array(data['odom_x'])
y1 = np.array(data['odom_y'])
x2 = np.array(data['odom_filter_x'])
y2 = np.array(data['odom_filter_y'])
x3 = np.array(data['cmd_x'])
y3 = np.array(data['cmd_y'])
x4 = (np.array(data['ground_x']) - 0.0553) * 0.929008008169053

y4 = (np.array(data['ground_y']) - 0.657) * 0.91784356538233


# plt.plot(x1, y1, marker='o', linestyle='', label='Dataset 1')
# plt.plot(x2, y2, marker='x', linestyle='', label='Dataset 2')

plt.plot(x1, y1, label='odom')
plt.plot(x2, y2, label='ekf')
plt.plot(x3, y3, label='cmd')
plt.plot(x4, y4, label='ground')

plt.xlabel('X Axis Label')
plt.ylabel('Y Axis Label')
plt.title('Circle Test Plot')
plt.legend()  # Show legend

ax.set_aspect(1)

plt.grid(True)

plt.show()