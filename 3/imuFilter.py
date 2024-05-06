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

bias_time = 5
accerlerometer_bias = [0,0,0]
bias_sum = [0, 0, 0]

t0 = t[0]
index = 0

while t[index] - t0 < bias_time:
    bias_sum[0] += ax[index]
    bias_sum[1] += ay[index]
    bias_sum[2] += az[index]
    index += 1

accerlerometer_bias[0] = bias_sum[0] / index
accerlerometer_bias[1] = bias_sum[1] / index
accerlerometer_bias[2] = bias_sum[2] / index

filtersize = 10

with open("filtered.json", "w") as file:
    for i in range(0, len(t)):
        
        if(i < filtersize):
            file.write("{\"msg\": \"imu_raw\", \"stamp\": " + str(t[i]) + ", \"ax\": " + str(ax[i] - accerlerometer_bias[0]) + ", \"ay\": " + str(ay[i] - accerlerometer_bias[1]) + ", \"az\": " + str(az[i] - accerlerometer_bias[2]) + "}\n")
            continue
        
        current_x = 0
        current_y = 0
        current_z = 0

        for j in range(0, 10):
            current_x += ax[i - j]
            current_y += ay[i - j]
            current_z += az[i - j]

        current_x = (current_x/filtersize) - accerlerometer_bias[0]
        current_y = (current_y/filtersize) - accerlerometer_bias[1]
        current_z = (current_z/filtersize) - accerlerometer_bias[2]

        file.write("{\"msg\": \"imu_raw\", \"stamp\": " + str(t[i]) + ", \"ax\": " + str(current_x) + ", \"ay\": " + str(current_y) + ", \"az\": " + str(current_z) + "}\n")
