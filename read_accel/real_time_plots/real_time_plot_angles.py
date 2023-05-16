import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time

# Establish a serial connection
ser = serial.Serial('COM6', 115200)  # substitute 'COM3' with your port number and '9600' with your baud rate
time.sleep(1)
# Variables to store data
pitch_a_data = []
roll_a_data = []
pitch_g_data = []
roll_g_data = []
pitch_f_data = []
roll_f_data = []

fig, ax = plt.subplots()

lines = []
colors = ['r', 'b', 'g', 'y', 'm', 'c']
labels = ['Pitch Accelerometer', 'Roll Accelerometer', 'Pitch Gyroscope', 'Roll Gyroscope', 'Pitch Filtered', 'Roll Filtered']

for i in range(6):
    line, = ax.plot([], [], colors[i], label=labels[i])
    lines.append(line)

ax.legend(loc='upper left')

# Set y-axis limits
ax.set_ylim([-90, 90])
ax.yaxis.grid(True, linestyle='--', which='major', color='grey', alpha=.25)

def init():
    for line in lines:
        line.set_data([], [])
    return lines

def update(i):
    while (ser.inWaiting() == 0):  # wait for data
        pass
    arduinoString = ser.readline().decode('utf-8')  # read a line from the serial port
    data = arduinoString.split()

    if len(data) == 6:  # check if data array contains all data
        pitch_a = float(data[0])
        roll_a = float(data[1])
        pitch_g = float(data[2])
        roll_g = float(data[3])
        pitch_f = float(data[4])
        roll_f = float(data[5])

        pitch_a_data.append(pitch_a)
        roll_a_data.append(roll_a)
        pitch_g_data.append(pitch_g)
        roll_g_data.append(roll_g)
        pitch_f_data.append(pitch_f)
        roll_f_data.append(roll_f)

        # Limit the size of the data list
        if len(pitch_a_data) > 50:
            pitch_a_data.pop(0)
        if len(roll_a_data) > 50:
            roll_a_data.pop(0)
        if len(pitch_g_data) > 50:
            pitch_g_data.pop(0)
        if len(roll_g_data) > 50:
            roll_g_data.pop(0)
        if len(pitch_f_data) > 50:
            pitch_f_data.pop(0)
        if len(roll_f_data) > 50:
            roll_f_data.pop(0)

    lines[0].set_data(range(len(pitch_a_data)), pitch_a_data)
    lines[1].set_data(range(len(roll_a_data)), roll_a_data)
    lines[2].set_data(range(len(pitch_g_data)), pitch_g_data)
    lines[3].set_data(range(len(roll_g_data)), roll_g_data)
    lines[4].set_data(range(len(pitch_f_data)), pitch_f_data)
    lines[5].set_data(range(len(roll_f_data)), roll_f_data)

    ax.relim()
    ax.autoscale_view()

    return lines


ani = animation.FuncAnimation(fig, update, init_func=init, blit=False, interval=20)
plt.show()
