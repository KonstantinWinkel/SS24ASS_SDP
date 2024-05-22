import json
import argparse
import numpy as np
import math
import matplotlib.pyplot as plt

begin = 2500
end = 5000

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

t, ax_p, ay_p, az_p, wx, wy, wz = readIMU(args.file)

# get initial Gravity
bias_time = 5
accerlerometer_bias = [0,0,0]
original_bias = [0,0,0]
bias_sum = [0, 0, 0]

t0 = t[0]
index = 0

while t[index] - t0 < bias_time:
    bias_sum[0] += ax_p[index]
    bias_sum[1] += ay_p[index]
    bias_sum[2] += az_p[index]
    index += 1

accerlerometer_bias[0] = bias_sum[0] / index
accerlerometer_bias[1] = bias_sum[1] / index
accerlerometer_bias[2] = bias_sum[2] / index


print(accerlerometer_bias)


#coppied from Rotation
wzs = [0]
for i in range(1,len(wz)):
    wzs.append(wzs[i-1]+(wz[i]*(t[i]-t[i-1])))

wys = [0]
for i in range(1,len(wy)):
    wys.append(wys[i-1]+(wy[i]*(t[i]-t[i-1])))

wxs = [0]
for i in range(1,len(wx)):
    wxs.append(wxs[i-1]+(wx[i]*(t[i]-t[i-1])))

print('Loaded ' + str(len(t)) + ' IMU messages.')

fig, axs = plt.subplots(3)
fig.suptitle("Angle xyz")
axs[0].scatter(t[begin : end], wxs[begin : end], s = 2)
axs[1].scatter(t[begin : end], wys[begin : end], s = 2)
axs[2].scatter(t[begin : end], wzs[begin : end], s = 2)

d = math.pi / 180

#preprocess a
for i in range(0, len(ax_p)):
    ax_p[i] -= accerlerometer_bias[1] * math.sin(d * wzs[i]) + accerlerometer_bias[0] * math.cos(d * wzs[i])
    ay_p[i] -= accerlerometer_bias[1] * math.cos(d * wzs[i]) - accerlerometer_bias[0] * math.sin(d * wzs[i])
    az_p[i] -= accerlerometer_bias[2] 

#filter a
filtersize = 100
ax = []
ay = []
az = []
for i in range(0, len(t)):
        
    # for no filter used:
    #ax.append(ax_p[i])
    #ay.append(ay_p[i])
    #az.append(az_p[i])
    #continue

    # for fitlering:
    if(i < filtersize):
        ax.append(ax_p[i])
        ay.append(ay_p[i])
        az.append(az_p[i])
        continue
    
    current_x = 0
    current_y = 0
    current_z = 0

    for j in range(0, filtersize):
        current_x += ax_p[i - j]
        current_y += ay_p[i - j]
        current_z += az_p[i - j]

    current_x = (current_x/filtersize)
    current_y = (current_y/filtersize)
    current_z = (current_z/filtersize)
    

    ax.append(current_x)
    ay.append(current_y)
    az.append(current_z)

axi = [0]
for i in range(1,len(ax)):
    axi.append(ax[i-1]+(ax[i]*(t[i]-t[i-1])))

ayi = [0]
for i in range(1,len(ay)):
    ayi.append(ay[i-1]+(ay[i]*(t[i]-t[i-1])))

azi = [0]
for i in range(1,len(az)):
    azi.append(az[i-1]+(az[i]*(t[i]-t[i-1])))

axii = [0]
for i in range(1,len(ax)):
    axii.append(axii[i-1]+(axi[i]*(t[i]-t[i-1]))+0.5*(ax[i]-ax[i-1]*(t[i]-t[i-1])*(t[i]-t[i-1])))

ayii = [0]
for i in range(1,len(ay)):
    ayii.append(ayii[i-1]+(ayi[i]*(t[i]-t[i-1]))+0.5*(ay[i]-ay[i-1]*(t[i]-t[i-1])*(t[i]-t[i-1])))

azii = [0]
for i in range(1,len(az)):
    azii.append(azii[i-1]+(azi[i]*(t[i]-t[i-1]))+0.5*(az[i]-az[i-1]*(t[i]-t[i-1])*(t[i]-t[i-1])))



fig2, axs2 = plt.subplots(3)
fig2.suptitle("Position xyz")
axs2[0].scatter(t[begin : end], axii[begin : end], s = 2)
axs2[1].scatter(t[begin : end], ayii[begin : end], s = 2)
axs2[2].scatter(t[begin : end], azii[begin : end], s = 2)
plt.show()