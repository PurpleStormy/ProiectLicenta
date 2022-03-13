from gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
from utils import plot_path
import numpy as np
from dijkstra import Solution


import itertools

import matplotlib.pyplot as plt

from scipy import misc
from scipy.sparse.dok import dok_matrix
from scipy.sparse.csgraph import dijkstra
from a_star2 import *
import sys

# csv = np.genfromtxt('obstacle.csv', delimiter=" ")
# point_list = []
# for i in range(0, csv.shape[0]):
#     for j in range(0, csv.shape[1]):
#         if csv[i][j] == 1:
#             point_list.append((i, j))
#
# point_list = np.asarray(point_list)
# fig = plt.figure()
# plt.ylim(0, csv.shape[0])
# plt.xlim(0, csv.shape[1])
# plt.axis([0, csv.shape[0], 0, csv.shape[1]])
# plt.scatter(point_list[:, 0], point_list[:, 1], marker=".", color="black", s=1)
# plt.show()
# fig.savefig('graphic_obstacles.png')


lab = [
    [0, 1, 1],
    [0, 1, 1],
    [0, 1, 1]
]

print(astar(lab, 0, 2, 2, 2))