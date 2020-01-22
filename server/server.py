from threading import Thread
import serial
import time
import collections
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import colors
import struct
import copy
import pandas as pd
import numpy as np
from numpy_ringbuffer import RingBuffer
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

        # give an array for each type of data and store them in a ring buffer of size len(self.data_types)
        self.data = RingBuffer(capacity = stack_size, dtype=(float, (len(self.data_types),)))
        self.data_to_append = np.zeros( shape=(len(self.data_types),) )

        self.is_run = True
        self.is_receiving = False
        self.thread = None
        self.plot_timer = 0
        self.previous_timer = 0
        # self.csvData = []

        # define binning for polar heatmap
        self.min_sensor_distance = 20
        self.max_sensor_distance = 200
        self.radius_bins_heatmap = np.linspace(self.min_sensor_distance,self.max_sensor_distance, 16)
        self.angle_bins_heatmap = np.linspace(-0.1,2*np.pi-0.1, 40)

        ### MIDI Settings ###
        self.mido_outports = mido.get_output_names()
        print("Trying to connect to MIDI port:", self.mido_outports[1])
        try:
            self.midi_outport = mido.open_output(self.mido_outports[1])
            print("Connected to MIDI port:", self.mido_outports[1])
        except IOError:
            print("Error connecting to MIDI port:", self.mido_outports[1])
            sys.exit(1)

        #[F2,G#2, C3, C#3, D#3, F3] C# Major scale
        # self.notes = [41, 44, 48, 49, 51, 53, 58, 60]
        # self.note_status = [False]*len(self.notes)

        #[F#2,G#2, C3, C#3, D#3, F3, G#3, A#4]
        self.notes = [42, 44, 48, 49, 51, 53, 58, 60]
        self.note_status = [False]*len(self.notes)

        print('\nTrying to connect to: ' + str(port_name) + ' at ' + str(baud_rate) + ' BAUD.')
        try:
            self.serial_connection = serial.Serial(port_name, baud_rate, timeout=4)
            print('Connected to ' + str(port_name) + ' at ' + str(baud_rate) + ' BAUD.\n')
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

    def get_serial_data(self, frame, ax, figNumber, maxDataLength, rects):

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
            self.data_to_append[i],  = struct.unpack(self.data_types[i], data_value)

        # print("\r",self.data_to_append,  end="\t")

        self.data.append(self.data_to_append)

        self.radius = self.data[:,0]
        self.angle = self.data[:,1]*np.pi/180

        if (figNumber == 1):

            ax.clear()            

            self.hist, _ , _ = np.histogram2d(self.angle, self.radius, bins=(self.angle_bins_heatmap, self.radius_bins_heatmap), density=True)
            self.A, self.R = np.meshgrid(self.angle_bins_heatmap, self.radius_bins_heatmap)

            self.pc_mesh = ax.pcolormesh(self.A, self.R, self.hist.T, cmap="magma")
           
        if (figNumber == 2):

            self.N_azimuth, self.bins_azimut = np.histogram(self.angle*180/np.pi, bins=range(-4,365-4,9))
         
            for rect, h in zip(rects[0],self.N_azimuth):
                rect.set_height(h)

            #automate to take the number of notes
            self.N_radius, self.bins_radius = np.histogram(self.radius, bins=np.linspace(20,200,8))

            if self.N_radius.sum() > 0:
                # self.fracs = self.N_radius/self.N_radius.sum()
                self.fracs = self.N_radius/maxDataLength
                for rect, h in zip(rects[1], self.fracs):
                    rect.set_height(h)   
            else:
                for rect in rects[1]:
                    rect.set_height(0)
                self.fracs = self.N_radius

            for i in range(0,len(self.fracs)):

                if (self.fracs[i] > 0.00001 ):
                    
                    if (self.note_status[i] == False):

                        self.midi_msg = mido.Message('note_on', note=self.notes[i], channel=i)
                        self.midi_outport.send(self.midi_msg)
                        # print("Note on", self.notes[i])
                        self.note_status[i] = True

                    self.midi_msg = mido.Message('control_change', channel=i, control=0, value=int(self.fracs[i]*127), time=0)
                    # self.midi_msg = mido.Message('control_change', channel=i, control=0, value=int(127), time=0)
                    self.midi_outport.send(self.midi_msg)
                    # print('CC channel',i+1,'value',int(self.fracs[i]*127))

                elif (self.fracs[i] < 0.00001 ):

                    if (self.note_status[i] == True):

                        self.midi_msg = mido.Message('note_off', note=self.notes[i], channel=i)
                        self.midi_outport.send(self.midi_msg)
                        # print("Note off", self.notes[i])
                        self.note_status[i] = False
                
    def close(self):
        self.is_run = False
        self.thread.join()
        self.serial_connection.close()
        self.midi_outport.close()
        print('Disconnected...')



def main():

    # port_name = 'COM6'
    port_name = 'COM9'
    # port_name = '/dev/ttyUSB0'

    baud_rate = 115200

    # Arduino sends a stream of data consisting of 1,...,num_data_bytes bytes.
    # The data is arranged to form a set of data_types, each one with length (in bytes) data_length 
    # A stack of size stack_size_data_points data points is stored.
    stack_size_data_points = 100             # number of real time data points
    data_types = ['H','H','f','h']          # ['H' 2 bytes unsigned short, 'f' 4 byte float]
    data_length = [2, 2, 4, 2]              # Length in bytes of each data type
    num_data_bytes = np.sum(data_length)    # number of bytes of one data stream
    
    s = serial_port_read(port_name, baud_rate, stack_size_data_points, num_data_bytes, data_types, data_length)   # initializes all required variables
    s.start_serial_reading()                                               # starts reading serial data in the background thread

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    # xmin = 0
    # xmax = stack_data_points
    # ymin = 0
    # ymax = 700

    fig = plt.figure(facecolor='k', figsize=(150,150))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_frame_on(False)
    ax.tick_params(axis='x', colors='black')
    ax.tick_params(axis='y', colors='black')
    ax.set_rmax(400)
    ax.set_rorigin(20)

    fig1 = plt.figure(facecolor='w', figsize=(400,800))
    ax1 = fig1.add_subplot(211)
    ax2 = fig1.add_subplot(212)

    hist, bins = np.histogram([],bins=range(-4,365-4,9))
    width = 0.75 * (bins[1] - bins[0])
    center = (bins[:-1] + bins[1:]) / 2

    rects1 = ax1.bar(center, hist, align='center', width=width)
    ax1.set_ylim(0,5)

    # automate to take the number of notes
    hist, bins = np.histogram([],bins=np.linspace(20,200,8))
    width = 0.75 * (bins[1] - bins[0])
    center = (bins[:-1] + bins[1:]) / 2

    rects2 = ax2.bar(center, hist, align='center', width=width)
    ax2.set_ylim(0,1)

    
    anim = animation.FuncAnimation(fig, s.get_serial_data, fargs=(ax, 1, stack_size_data_points, (rects1, rects2)), interval=pltInterval)    # fargs has to be a tuple
    anim1 = animation.FuncAnimation(fig1, s.get_serial_data, fargs=((ax1,ax2), 2, stack_size_data_points, (rects1, rects2)), interval=pltInterval)    # fargs has to be a tuple


    plt.show()

    s.close()


if __name__ == '__main__':
    main()