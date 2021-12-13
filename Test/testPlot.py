import numpy as np
from matplotlib import pyplot as plt
from rplidar import RPLidar


def get_data():
    lidar = RPLidar('COM3', baudrate=115200)
    for scan in lidar.iter_scans(max_buf_meas=500):
        break
    lidar.stop()
    return scan


if __name__ == '__main__':
    for i in range(1000000):
        if i % 7 == 0:
            x = []
            y = []
        print(i)
        current_data = get_data()
        for point in current_data:
            if point[0] == 15:
                radian = 3.14/point[1]
                x.append(point[2] * np.sin(radian))
                y.append(point[2] * np.cos(radian))
        plt.clf()
        plt.scatter(x, y)
        plt.pause(.1)
    plt.show()
