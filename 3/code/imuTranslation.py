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

    return t, ax, ay, az, wx, wy, wz

parser = argparse.ArgumentParser(description='Process IMU log.')
parser.add_argument(dest='file', type=str, help='Input file with IMU JSON messages.')
args = parser.parse_args()

t, ax, ay, az, wx, wy, wz = readIMU(args.file)

print('Loaded ' + str(len(t)) + ' IMU messages.')

#velocities (unused)
axi = [0]
for i in range(1,len(ax)):
    axi.append(ax[i-1]+(ax[i]*(t[i]-t[i-1])))

ayi = [0]
for i in range(1,len(ay)):
    ayi.append(ay[i-1]+(ay[i]*(t[i]-t[i-1])))

azi = [0]
for i in range(1,len(az)):
    azi.append(az[i-1]+(az[i]*(t[i]-t[i-1])))

#positions
axii = [0]
for i in range(1,len(ax)):
    axii.append(axii[i-1]+(axi[i]*(t[i]-t[i-1]))+0.5*(ax[i]-ax[i-1]*(t[i]-t[i-1])*(t[i]-t[i-1])))

ayii = [0]
for i in range(1,len(ay)):
    ayii.append(ayii[i-1]+(ayi[i]*(t[i]-t[i-1]))+0.5*(ay[i]-ay[i-1]*(t[i]-t[i-1])*(t[i]-t[i-1])))

azii = [0]
for i in range(1,len(az)):
    azii.append(azii[i-1]+(azi[i]*(t[i]-t[i-1]))+0.5*(az[i]-az[i-1]*(t[i]-t[i-1])*(t[i]-t[i-1])))


fig, axs = plt.subplots(3)
fig.suptitle("Position xyz")
axs[0].scatter(t, axii, s = 2)
axs[1].scatter(t, ayii, s = 2)
axs[2].scatter(t, azii, s = 2)
plt.show()