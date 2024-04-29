import os
import serial 
import json
import math
import matplotlib.pyplot as plt

# load config file (same directory as this file)
configFile = open(os.path.dirname(os.path.abspath(__file__)) + '/../../config.json')
config = json.load(configFile)

print("Opening port " + str(config["serial_port"]) + " with baudrate " + str(config["serial_baudrate"]) + ".")

# open serial port
sensorcube = serial.Serial(port=config["serial_port"], baudrate=config["serial_baudrate"])

last_heading = -999999
record = []
while True:
    try:
        #print(sensorcube.readline().decode('utf-8').rstrip())
        line = sensorcube.readline()
        data = json.loads(line)
        if not "msg" in data:
            continue
        if data["msg"] == "imu_euler":
            last_heading = float(data["heading"])
        elif data["msg"] == "tof_raw":
            if last_heading == -999999:
                continue
            if int(data["objects"]) == 0:
                pass
                #record.append((last_heading, 10))
            else:
                record.append((last_heading, float(data["range"][0])))
    except KeyboardInterrupt:
        break


processed_x = []
processed_y = []

for r in record:
    processed_x.append(math.cos(r[0] * (math.pi/180)) * (r[1]+0.05))
    processed_y.append(math.sin(r[1] * (math.pi/180)) * (r[1]+0.05))

plt.scatter(processed_x, processed_y, s = 2)
plt.show()