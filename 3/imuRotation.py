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

    return t, ax, ay, az, wx, wy, wz

parser = argparse.ArgumentParser(description='Process IMU log.')
parser.add_argument(dest='file', type=str, help='Input file with IMU JSON messages.')
args = parser.parse_args()

t, ax, ay, az, wx, wy, wz = readIMU(args.file)

print('Loaded ' + str(len(t)) + ' IMU messages.')

# process IMU data
wzs = [0]
for i in range(1,len(wz)):
    wzs.append(wzs[i-1]+(wz[i]*(t[i]-t[i-1])))

#stay at 0°
#go to -90°
#go to 90°
#go to -30°
plt.title("Position as angle(deg) over time")
plt.scatter(t, wzs, s = 2)
plt.show()