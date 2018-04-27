#!usr/bin/env python  
#coding=utf-8  

import pyaudio
import rospy
from os.path import join
from rospkg import RosPack
import os
import wave

class AudioHelp(object):
    def __init__(self):
        self.rospack = RosPack()
        # Sound setting
        #define stream chunk   
        self.chunk = 1024  
        #open a wav format music  
        self.help_msg = dict()
        #instantiate PyAudio  
        self.p = pyaudio.PyAudio()

    def play(self,msg_name):
        #import wave
        #read data 
        f = wave.open(join(self.rospack.get_path('pobax_playground'), 'config', "help_"+msg_name+".wav"),"rb")
        data = f.readframes(self.chunk)
        stream = self.p.open(format = self.p.get_format_from_width(f.getsampwidth()),  
            channels = f.getnchannels(),  
            rate = f.getframerate(),  
            output = True) 
        #play stream  
        while data:  
            stream.write(data)  
            data = f.readframes(self.chunk)  
        #stop stream  
        stream.stop_stream() 
        stream.close()

        