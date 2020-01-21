from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
import struct
import copy
import pandas as pd
import numpy as np


class serialPlot:
    def __init__(self, serialPort='COM6', serialBaud=9600, plotLength=100, dataNumBytes=2, numPlots=1):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        self.dataNumBytes = dataNumBytes
        self.numPlots = numPlots
        self.rawData = bytearray(numPlots * dataNumBytes)
        self.dataType = None

        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float

        self.data = []

        for i in range(numPlots):   # give an array for each type of data and store them in a list
            self.data.append(collections.deque([0] * plotLength, maxlen=plotLength))

        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        # self.csvData = []

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialData(self, frame, ax, lines, lineValueText, lineLabel, timeText):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
        privateData = copy.deepcopy(self.rawData[:])    # so that the 3 values in our plots will be synchronized to the same sample time
        
        ax.clear()

        for i in range(self.numPlots):
            data = privateData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
            value,  = struct.unpack('H', data)
            self.data[i].append(value)    # we get the latest data point and append it to our array
            # lines[i].set_data(range(self.plotMaxLength), self.data[i])
            # lineValueText[i].set_text('[' + lineLabel[i] + '] = ' + str(value))
        # lines[0].set_data(self.data[1], self.data[0])
        print(self.data[0][-1],self.data[1][-1])

        self.azimut = np.array(self.data[1])*np.pi/180
        self.radius = np.array(self.data[0])

        # define binning
        self.rbins = np.linspace(0,400, 40)
        self.abins = np.linspace(-0.1,2*np.pi-0.1, 40)

        self.hist, _, _ = np.histogram2d(self.azimut, self.radius, bins=(self.abins, self.rbins))
        self.A, self.R = np.meshgrid(self.abins, self.rbins)

        # ax.hist(self.data[1],bins=range(4,364+9,9))
        self.pc = ax.pcolormesh(self.A, self.R, self.hist.T, cmap="magma")
        ax.plot(self.azimut[-1],self.radius[-1],'ow', markersize = 5)
        ax.set_rmax(400)
        ax.set_rorigin(20)

        # self.csvData.append([self.data[0][-1], self.data[1][-1], self.data[2][-1]])

    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.serialConnection.readinto(self.rawData)
            self.isReceiving = True
            # print(self.rawData)

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')


def main():
    portName = 'COM10'
    # portName = 'COM6'
    # portName = '/dev/ttyUSB0'
    baudRate = 9600
    maxPlotLength = 500     # number of points in x-axis of real time plot
    dataNumBytes = 2        # number of bytes of 1 data point
    numPlots = 2            # number of plots in 1 graph
    s = serialPlot(portName, baudRate, maxPlotLength, dataNumBytes, numPlots)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = 0
    ymax = 700
    # fig, ax = plt.subplots(subplot_kw=dict(projection="polar"),figsize = (10,10), facecolor='black')
    # fig = plt.figure(figsize=(10, 8))
    # ax = plt.axes()#(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    # ax.set_title('Sonar')
    # ax.set_xlabel("Angle")
    # ax.set_ylabel("Distance")
    # ax.set_frame_on(False)

    fig = plt.figure(facecolor='k', figsize=(500,500))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_frame_on(False)


    lineLabel = ['X', 'Y']
    style = ['ro', 'co']  # linestyles for the different plots
    timeText = ax.text(0.70, 0.95, '', transform=ax.transAxes)
    lines = []
    lineValueText = []

    # for i in range(numPlots):
    #     lines.append(ax.plot([], [], style[i], label=lineLabel[i])[0])
    #     lineValueText.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))

    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(ax, lines, lineValueText, lineLabel, timeText), interval=pltInterval)    # fargs has to be a tuple

    # plt.legend(loc="upper left")
    plt.show()

    s.close()


if __name__ == '__main__':
    main()