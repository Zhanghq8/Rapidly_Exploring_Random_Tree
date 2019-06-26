#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

path = np.loadtxt('path.txt')
# print (path[0])
smoothpath = np.loadtxt('smoothpath.txt')
# print (smoothpath.shape)


x_list = path[0].tolist()
y_list = path[1].tolist()

x_list_smoothed = smoothpath[0].tolist()
y_list_smoothed = smoothpath[1].tolist()

plt.figure('Path figure')

ax = plt.gca()
ax.add_patch(
	patches.Rectangle(
		(15, 20),   # (x,y)
		5,          # width
		20,          # height
		label='Obstacle'
	)	
)
ax.add_patch(
	patches.Rectangle(
		(30, 0),   # (x,y)
		5,          # width
		30,          # height
		label='Obstacle'
	)	
)




ax.set_xlim(left=0, right=50)
ax.set_ylim(bottom=0, top=50)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('RRT Result')

ax.plot(x_list, y_list, color='r', marker='o', markersize=5, linewidth=2, alpha=0.6, label='Path without smooth')
ax.plot(x_list_smoothed, y_list_smoothed, color='g', marker='*', markersize=5, linewidth=2, alpha=0.6, label='Path after smooth')
ax.plot([path[0][0]], [path[1][0]], 'v', color='k', markersize=10, label='Start')
ax.plot([path[0][-1]], [path[1][-1]], '^', color='k', markersize=10, label='Goal')
ax.grid()

ax.legend()
plt.show()

