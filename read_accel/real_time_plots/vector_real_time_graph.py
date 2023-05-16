import matplotlib.pyplot as plt
import numpy as np
import serial
import matplotlib.animation as animation

# Open the serial port
ser = serial.Serial('COM6', 115200)

# Create the figure and a polar subplot
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)

# Create two quiver plots with different colors
Q1 = ax.quiver(0, 0, 0, 0, color='b', label='Pitch')
Q2 = ax.quiver(0, 0, 0, 0, color='r', label='Roll')

# Set the radius limits
ax.set_rlim(0, 1)

# Set the title for the subplot
ax.set_title('Pitch and Roll')

# Add legend
ax.legend()


def update(i):
    while (ser.inWaiting() == 0):  # wait for data
        pass
    arduinoString = ser.readline().decode('utf-8')  # read a line from the serial port
    data = arduinoString.split()

    if len(data) == 6:  # check if data array contains all data
        pitch = float(data[4])
        roll = float(data[5])

        # Convert angles from degrees to radians and adjust for polar plot
        pitch_rad = np.deg2rad(pitch)
        roll_rad = np.deg2rad(roll)

        # Calculate the x and y components of the vectors
        pitch_x = np.cos(pitch_rad)
        pitch_y = np.sin(pitch_rad)
        roll_x = np.cos(roll_rad)
        roll_y = np.sin(roll_rad)

        # Update the quiver data
        Q1.set_UVC(pitch_x, pitch_y)
        Q2.set_UVC(roll_x, roll_y)

    return Q1, Q2,


# Create the animation
ani = animation.FuncAnimation(fig, update, interval=10)

plt.show()
