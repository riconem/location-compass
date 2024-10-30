import serial
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
DATA_FRAME = 100

serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)  # Replace with your port
plt.ion()  # Interactive mode on

# Create figure for plotting
fig, ax = plt.subplots()

timedata, xdata, ydata, zdata = [], [], [], []  # Data arrays to store incoming data

# Update the plot with new data
def update(frame):
    # Read data from serial port
    line = serial_port.readline().decode('utf-8').strip()

    values = line.split(",")
    print(values)

    try:
        time = float(values[0])
    except:
        time = None

    if len(values) == 4 and time:
        timedata.append(float(values[0]))
        xdata.append(int(values[1]))
        ydata.append(int(values[2]))
        zdata.append(int(values[3]))


    # Keep the graph clean, show only the last 50 points
    ax.clear()
    ax.plot(timedata[-DATA_FRAME:], xdata[-DATA_FRAME:], label="x")
    ax.plot(timedata[-DATA_FRAME:], ydata[-DATA_FRAME:], label="y")
    ax.plot(timedata[-DATA_FRAME:], zdata[-DATA_FRAME:], label="z")

    # # Set axis limits
    ax.set_ylim(-2000, 2000)
    # ax.set_xlim(max(0, len(ydata) - 50), len(ydata))

    ax.set_xlabel('Time')
    ax.set_ylabel('Compass Data')

# Use matplotlib's FuncAnimation to continuously update the plot
ani = FuncAnimation(fig, update, interval=10)  # Update every 10 ms

# Show the plot
plt.show()

# Keep the program running until interrupted
try:
    while True:
        plt.pause(0.1)  # Keep the plot window responsive
except KeyboardInterrupt:
    print("Program interrupted")

# Close the serial port when done
serial_port.close()
