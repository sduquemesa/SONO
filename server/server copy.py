from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import colors
import struct
import copy
import pandas as pd
import numpy as np
from scipy import interpolate
import mido


class serialPlot:

    def __init__(self, serialPort='COM6', serialBaud=9600, dataLength=100, dataNumBytes=2, numData=1):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = dataLength
        self.dataNumBytes = dataNumBytes
        self.numData = numData
        self.rawData = bytearray(numData * dataNumBytes)
        self.dataType = None

        self.midoOutports = mido.get_output_names()
        print("Connecting to MIDI port:", self.midoOutports[1])
        self.midiOutport = mido.open_output(self.midoOutports[1])

        #[F2,G#2, C3, C#3, D#3, F3, G#3]
        self.notes = [41, 44, 48, 49, 51, 53, 56]
        self.note_status = [False, False, False, False, False, False, False]

        if dataNumBytes == 2:
            self.dataType = 'H'     # 2 byte integer unsigned
        elif dataNumBytes == 4:
            self.dataType = 'F'     # 4 byte float unsigned

        self.data = []

        for i in range(numData):   # give an array for each type of data and store them in a list
            self.data.append(collections.deque([0] * dataLength, maxlen=dataLength))

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

    def getSerialData(self, frame, ax, fig, figNumber, maxDataLength):

        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
        self.previousTimer = currentTimer

        privateData = copy.deepcopy(self.rawData[:])    # so that the 3 values in our plots will be synchronized to the same sample time
        
        # unpack and decode incoming data and add to variable data
        for i in range(self.numData):
            data = privateData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
            value,  = struct.unpack(self.dataType, data)
            self.data[i].append(value)    # we get the latest data point and append it to our array
        # print("\r",self.data[1][-1],"\t",self.data[0][-1], end="")

        self.azimut = np.asarray(self.data[1])*np.pi/180
        self.radius = np.asarray(self.data[0])

        if (figNumber == 1):

            plt.figure(fig.number)
            ax.clear()            

            # define binning: 0m to 4m with steps of 12.5cm (32 steps)
            self.rbins = np.linspace(0,400, 40)
            self.abins = np.linspace(-0.1,2*np.pi-0.1, 40)

            self.hist, _ , _ = np.histogram2d(self.azimut, self.radius, bins=(self.abins, self.rbins), density=True)
            self.A, self.R = np.meshgrid(self.abins, self.rbins)

            self.pc = ax.pcolormesh(self.A, self.R, self.hist.T, cmap="magma")
            
            # self.interp_hist = interpolate.interp2d(self.abins[:-1],self.rbins[:-1],self.hist.T,kind='linear')

            # define interpolation binning
            # self.rbins_interp = np.linspace(20,400, 40*4)
            # self.abins_interp = np.linspace(0.0,2*np.pi, 40*4)
            # self.A_interp, self.R_interp = np.meshgrid(self.abins_interp, self.rbins_interp)

            # self.hist_interp = self.interp_hist(self.abins_interp,self.rbins_interp)

            # self.pc = ax.pcolormesh(self.A_interp, self.R_interp, self.hist_interp, cmap="magma")
            
            # ax.set_rmax(400)
            ax.set_rorigin(20)

        if (figNumber == 2):

            plt.figure(fig.number)
             
            ax[0].clear()
            ax[1].clear()

            # self.weights_radius = np.ones_like(self.radius)/maxDataLength
            self.weights_radius = np.ones_like(self.radius)/np.max(self.radius)

            self.N_azimuth, self.bins_azimut, self.patches_azimuth = ax[0].hist(self.data[1],bins=range(-4,365-4,9))
            self.N_radius, self.bins_radius, self.patches_radius = ax[1].hist(self.radius,bins=np.linspace(20,300,8), weights=self.weights_radius)
            ax[1].set_ylim(0,1)


            # We'll color code by height, but you could use any scalar
            self.fracs = self.N_radius

            # we need to normalize the data to 0..1 for the full range of the colormap
            self.norm = colors.Normalize(self.fracs.min(), self.fracs.max())

            # Now, we'll loop through our objects and set the color of each accordingly
            for thisfrac, thispatch in zip(self.fracs, self.patches_radius):
                color = plt.cm.gist_yarg(self.norm(thisfrac))
                thispatch.set_facecolor(color)

            

            for i in range(0,np.shape(self.fracs)[0]):

                if (self.fracs[i] > 0.00001 ):
                    
                

                    self.midi_msg = mido.Message('note_on', note=self.notes[i], channel=i)
                    self.midiOutport.send(self.midi_msg)
                    print("Note on", self.notes[i])
                    self.note_status[i] = True
                    self.midi_msg = mido.Message('note_off', note=self.notes[i], channel=i)
                    time.sleep(0.5)
                    self.midiOutport.send(self.midi_msg)

                    # self.midi_msg = mido.Message('control_change', channel=i, control=0, value=int(self.N_radius[i]*127), time=0)
                    # self.midi_msg = mido.Message('control_change', channel=i, control=0, value=int(127), time=0)
                    # self.midiOutport.send(self.midi_msg)
                    # print('CC channel',i+1,'value',int(self.N_radius[i]*127))

                elif (self.fracs[i] < 0.00001 ):

                

                    self.midi_msg = mido.Message('note_off', note=self.notes[i], channel=i)
                    self.midiOutport.send(self.midi_msg)
                    # print("Note off", self.notes[i])
                    self.note_status[i] = False
            




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

    # portName = 'COM10'
    portName = 'COM6'
    # portName = '/dev/ttyUSB0'

    baudRate = 115200

    # Arduino sends a stream of data consisting of 1,...,numData information classes,
    # each one with a length of dataNumBytes. A stack of maxDataLength data poinsts is stored.
    maxDataLength = 100     # number of real time data points
    dataNumBytes = 2        # number of bytes of 1 data point
    numData = 2             # number of data information classes in 1 datapoint
    
    s = serialPlot(portName, baudRate, maxDataLength, dataNumBytes, numData)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxDataLength
    ymin = 0
    ymax = 700


    fig = plt.figure(facecolor='k', figsize=(1500,1500))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_frame_on(False)
    ax.tick_params(axis='x', colors='white')
    ax.tick_params(axis='y', colors='white')

    fig1 = plt.figure(facecolor='w', figsize=(400,800))
    ax1 = fig1.add_subplot(211)
    ax2 = fig1.add_subplot(212)

    
    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(ax, fig, 1, maxDataLength), interval=pltInterval)    # fargs has to be a tuple
    anim1 = animation.FuncAnimation(fig1, s.getSerialData, fargs=((ax1,ax2), fig1, 2, maxDataLength), interval=pltInterval)    # fargs has to be a tuple

    plt.show()

    s.close()


if __name__ == '__main__':
    main()