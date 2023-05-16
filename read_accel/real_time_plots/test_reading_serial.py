import serial
import numpy as np
import time

# Establish a serial connection
arduinoData = serial.Serial('COM6', 115200)  # substitute 'COM3' with your port number and '9600' with your baud rate
time.sleep(1)
while(True):
    while (arduinoData.inWaiting() == 0):
        pass
    dataPacket = arduinoData.readline()
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(' ')
    pitch_comp = splitPacket[4]
    roll_comp  = splitPacket[5]
    print(f"PITCH = {pitch_comp}")
    print(f"ROLL  = {roll_comp}\n")