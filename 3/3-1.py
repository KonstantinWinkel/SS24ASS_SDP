import json
import argparse
import numpy as np
import matplotlib.pyplot as plt

def readIMU(infile):
    t = []
    ax = []
    ay = []
    az = []
    wx = []
    wy = []
    wz = []

    file = open(infile, 'r')

    while True:
        line = file.readline()

        if not line:
            break

        try:
            data = json.loads(line)
        except:
            continue

        if not "msg" in data:
            continue

        if data["msg"] == "imu_raw":
            t.append(float(data["stamp"]))
            ax.append(float(data["ax"]))
            ay.append(float(data["ay"]))
            az.append(float(data["az"]))
            wx.append(float(data["wx"]))
            wy.append(float(data["wy"]))
            wz.append(float(data["wz"]))

    return np.array(t), np.array(ax), np.array(ay), np.array(az), np.array(wx), np.array(wy), np.array(wz)

parser = argparse.ArgumentParser(description='Process IMU log.')
parser.add_argument(dest='file', type=str, help='Input file with IMU JSON messages.')
args = parser.parse_args()

t, ax, ay, az, wx, wy, wz = readIMU(args.file)

print('Loaded ' + str(len(t)) + ' IMU messages.')

# process IMU data

plt.scatter(t, ax, s = 2)
plt.show()