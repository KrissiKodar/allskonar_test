import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Establish a serial connection
ser = serial.Serial('COM6', 115200)  # substitute 'COM3' with your port number and '9600' with your baud rate

# Variables to store data
accelerometer_data = {'x': [], 'y': [], 'z': []}
gyroscope_data = {'x': [], 'y': [], 'z': []}

fig1, ax1 = plt.subplots()
fig2, ax2 = plt.subplots()
# Set y-axis limits
ax1.set_ylim([-3*9.8, 3*9.8])  # for accelerometer
ax2.set_ylim([-200, 200])  # for gyroscope
lines = []
colors = ['r', 'g', 'b', 'c', 'm', 'y']
labels = ['Accelerometer X', 'Accelerometer Y', 'Accelerometer Z', 'Gyroscope X', 'Gyroscope Y', 'Gyroscope Z']

for i in range(6):
    if i < 3:  # first 3 lines for accelerometer
        line, = ax1.plot([], [], colors[i], label=labels[i])
    else:  # next 3 lines for gyroscope
        line, = ax2.plot([], [], colors[i], label=labels[i])
    lines.append(line)

ax1.legend(loc='upper left')
ax2.legend(loc='upper left')

def init():
    for line in lines:
        line.set_data([], [])
    return lines

def update(i):
    while (ser.inWaiting() == 0):  # wait for data
        pass
    arduinoString = ser.readline().decode('utf-8')  # read a line from the serial port
    data = arduinoString.split()

    if len(data) == 4:
        sensor = data[0]
        x, y, z = map(float, data[1:])

        if sensor == 'a':
            accelerometer_data['x'].append(x)
            accelerometer_data['y'].append(y)
            accelerometer_data['z'].append(z)
        elif sensor == 'g':
            gyroscope_data['x'].append(x)
            gyroscope_data['y'].append(y)
            gyroscope_data['z'].append(z)

        # Limit the size of the data list
        if len(accelerometer_data['x']) > 50:
            accelerometer_data['x'].pop(0)
            accelerometer_data['y'].pop(0)
            accelerometer_data['z'].pop(0)
        if len(gyroscope_data['x']) > 50:
            gyroscope_data['x'].pop(0)
            gyroscope_data['y'].pop(0)
            gyroscope_data['z'].pop(0)

    lines[0].set_data(range(len(accelerometer_data['x'])), accelerometer_data['x'])
    lines[1].set_data(range(len(accelerometer_data['y'])), accelerometer_data['y'])
    lines[2].set_data(range(len(accelerometer_data['z'])), accelerometer_data['z'])
    lines[3].set_data(range(len(gyroscope_data['x'])), gyroscope_data['x'])
    lines[4].set_data(range(len(gyroscope_data['y'])), gyroscope_data['y'])
    lines[5].set_data(range(len(gyroscope_data['z'])), gyroscope_data['z'])

    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()

    return lines

ani1 = animation.FuncAnimation(fig1, update, init_func=init, blit=False, interval=100)
ani2 = animation.FuncAnimation(fig2, update, init_func=init, blit=False, interval=100)
plt.show()
