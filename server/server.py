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
import sys
import itertools


class serial_port_read:

    def __init__(self, port_name = 'COM6', baud_rate = 9600, stack_size = 100, num_data_bytes = 2, data_types = [], data_length = []):

        self.port = port_name
        self.baud = baud_rate
        self.stack_size = stack_size
        self.num_data_bytes = num_data_bytes
        self.data_types = data_types
        self.data_length = data_length
        # self.numData = numData              # TO ERASE!
        self.rawdata_bytes = bytearray(num_data_bytes)

        self.data = []

        for i in data_types:   # give an array for each type of data and store them in a list
            self.data.append(collections.deque([0] * stack_size, maxlen=stack_size))

        self.is_run = True
        self.is_receiving = False
        self.thread = None
        self.plot_timer = 0
        self.previous_timer = 0
        # self.csvData = []

        ### MIDI Settings ###
        self.mido_outports = mido.get_output_names()
        print("Trying to connect to MIDI port:", self.mido_outports[1])
        try:
            self.midi_outport = mido.open_output(self.mido_outports[1])
            print("Connected to MIDI port:", self.mido_outports[1])
        except IOError:
            print("Error connecting to MIDI port:", self.mido_outports[1])
            sys.exit(1)

        #[F2,G#2, C3, C#3, D#3, F3, G#3]
        self.notes = [41, 44, 48, 49, 51, 53, 56]
        self.note_status = [False, False, False, False, False, False, False]


        print('Trying to connect to: ' + str(port_name) + ' at ' + str(baud_rate) + ' BAUD.')
        try:
            self.serial_connection = serial.Serial(port_name, baud_rate, timeout=4)
            print('Connected to ' + str(port_name) + ' at ' + str(baud_rate) + ' BAUD.')
        except:
            print("Failed to connect with " + str(port_name) + ' at ' + str(baud_rate) + ' BAUD.')
            sys.exit(1)

    def start_serial_reading(self):
        if self.thread == None:
            self.thread = Thread(target=self.background_thread)
            self.thread.start()
            # Block till we start receiving values
            while self.is_receiving != True:
                time.sleep(0.1)

    def background_thread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serial_connection.reset_input_buffer()
        while (self.is_run):
            self.serial_connection.readinto(self.rawdata_bytes)
            self.is_receiving = True
            # print(self.rawdata_bytes)

    def get_serial_data(self, frame, ax, fig, figNumber, maxDataLength):

        current_timer = time.perf_counter()
        self.plot_timer = int((current_timer - self.previous_timer) * 1000)     # the first reading will be erroneous
        self.previous_timer = current_timer

        static_data = copy.deepcopy(self.rawdata_bytes[:])    # so that the data values in the plots will be synchronized to the same sample time
        
        # unpack and decode incoming data and add to variable data
        if len(self.data_types) != len(self.data_length):
            raise ValueError('data_types and data_length lists have different length')

        data_length_cumsum = list(itertools.accumulate([0] + self.data_length))
        for i in range(len(self.data_types)):
            data_value = static_data[(data_length_cumsum[i]):(data_length_cumsum[i+1])]
            value,  = struct.unpack(self.data_types[i], data_value)
            self.data[i].append(value)    # we get the latest data point and append it to our deque in the data array
            print(value,  end="\t")
        # print("\r",self.data[1][-1],"\t",self.data[0][-1], end="")
        print("\r",end="")

        # self.azimut = np.asarray(self.data[1])*np.pi/180
        # self.radius = np.asarray(self.data[0])

        # if (figNumber == 1):

        #     plt.figure(fig.number)
        #     ax.clear()            

        #     # define binning: 0m to 4m with steps of 12.5cm (32 steps)
        #     self.rbins = np.linspace(0,400, 40)
        #     self.abins = np.linspace(-0.1,2*np.pi-0.1, 40)

        #     self.hist, _ , _ = np.histogram2d(self.azimut, self.radius, bins=(self.abins, self.rbins), density=True)
        #     self.A, self.R = np.meshgrid(self.abins, self.rbins)

        #     self.pc = ax.pcolormesh(self.A, self.R, self.hist.T, cmap="magma")
            
            # self.interp_hist = interpolate.interp2d(self.abins[:-1],self.rbins[:-1],self.hist.T,kind='linear')

            # define interpolation binning
            # self.rbins_interp = np.linspace(20,400, 40*4)
            # self.abins_interp = np.linspace(0.0,2*np.pi, 40*4)
            # self.A_interp, self.R_interp = np.meshgrid(self.abins_interp, self.rbins_interp)

            # self.hist_interp = self.interp_hist(self.abins_interp,self.rbins_interp)

            # self.pc = ax.pcolormesh(self.A_interp, self.R_interp, self.hist_interp, cmap="magma")
            
            # ax.set_rmax(400)
        #     ax.set_rorigin(20)

        # if (figNumber == 2):

        #     plt.figure(fig.number)
             
        #     ax[0].clear()
        #     ax[1].clear()

        #     # self.weights_radius = np.ones_like(self.radius)/maxDataLength
        #     self.weights_radius = np.ones_like(self.radius)/np.max(self.radius)

        #     self.N_azimuth, self.bins_azimut, self.patches_azimuth = ax[0].hist(self.data[1],bins=range(-4,365-4,9))
        #     self.N_radius, self.bins_radius, self.patches_radius = ax[1].hist(self.radius,bins=np.linspace(20,300,8), weights=self.weights_radius)
        #     ax[1].set_ylim(0,1)


        #     # We'll color code by height, but you could use any scalar
        #     self.fracs = self.N_radius

        #     # we need to normalize the data to 0..1 for the full range of the colormap
        #     self.norm = colors.Normalize(self.fracs.min(), self.fracs.max())

        #     # Now, we'll loop through our objects and set the color of each accordingly
        #     for thisfrac, thispatch in zip(self.fracs, self.patches_radius):
        #         color = plt.cm.gist_yarg(self.norm(thisfrac))
        #         thispatch.set_facecolor(color)

            

        #     for i in range(0,np.shape(self.fracs)[0]):

        #         if (self.fracs[i] > 0.00001 ):
                    
        #             if (self.note_status[i] == False):

        #                 self.midi_msg = mido.Message('note_on', note=self.notes[i], channel=i)
        #                 self.midiOutport.send(self.midi_msg)
        #                 print("Note on", self.notes[i])
        #                 self.note_status[i] = True

        #             # self.midi_msg = mido.Message('control_change', channel=i, control=0, value=int(self.N_radius[i]*127), time=0)
        #             self.midi_msg = mido.Message('control_change', channel=i, control=0, value=int(127), time=0)
        #             # self.midiOutport.send(self.midi_msg)
        #             # print('CC channel',i+1,'value',int(self.N_radius[i]*127))

        #         elif (self.fracs[i] < 0.00001 ):

        #             if (self.note_status[i] == True):

        #                 self.midi_msg = mido.Message('note_off', note=self.notes[i], channel=i)
        #                 self.midiOutport.send(self.midi_msg)
        #                 # print("Note off", self.notes[i])
        #                 self.note_status[i] = False
                
    def close(self):
        self.is_run = False
        self.thread.join()
        self.serial_connection.close()
        self.midi_outport.close()
        print('Disconnected...')



def main():

    # port_name = 'COM10'
    port_name = 'COM9'
    # port_name = '/dev/ttyUSB0'

    baud_rate = 115200

    # Arduino sends a stream of data consisting of 1,...,num_data_bytes bytes.
    # The data is arranged to form a set of data_types, each one with length (in bytes) data_length 
    # A stack of size stack_size_data_points data points is stored.
    stack_size_data_points = 50             # number of real time data points
    data_types = ['H','H','f','h']          # ['H' 2 bytes unsigned short, 'f' 4 byte float]
    data_length = [2, 2, 4, 2]              # Length in bytes of each data type
    num_data_bytes = np.sum(data_length)    # number of bytes of one data stream
    
    s = serial_port_read(port_name, baud_rate, stack_size_data_points, num_data_bytes, data_types, data_length)   # initializes all required variables
    s.start_serial_reading()                                               # starts background thread

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    # xmin = 0
    # xmax = stack_data_points
    # ymin = 0
    # ymax = 700


    fig = plt.figure(facecolor='k', figsize=(1500,1500))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_frame_on(False)
    ax.tick_params(axis='x', colors='white')
    ax.tick_params(axis='y', colors='white')

    # fig1 = plt.figure(facecolor='w', figsize=(400,800))
    # ax1 = fig1.add_subplot(211)
    # ax2 = fig1.add_subplot(212)

    
    anim = animation.FuncAnimation(fig, s.get_serial_data, fargs=(ax, fig, 1, stack_size_data_points), interval=pltInterval)    # fargs has to be a tuple
    # anim1 = animation.FuncAnimation(fig1, s.get_serial_data, fargs=((ax1,ax2), fig1, 2, stack_data_points), interval=pltInterval)    # fargs has to be a tuple

    plt.show()

    s.close()


if __name__ == '__main__':
    main()