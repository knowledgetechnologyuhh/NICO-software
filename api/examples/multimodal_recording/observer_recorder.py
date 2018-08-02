# -*- coding: utf-8 -*-
"""
Created on Wed Jul  4 10:42:18 2018

Based on flaskcom test class: https://git.informatik.uni-hamburg.de/twiefel/flaskcom/blob/master/test_class.py
@author: twiefel
@author: strahl
"""

import subprocess
import signal
import datetime

class Observer_Recorder:
    def __init__(self,topic_to_record,filename="",fps=30,codec="X264",depth_range=0):
        self.timed_filename=False
        if filename=="":
            self.timed_filename=True
            self.filename=""
        else:
            self.filename = filename
        self.topic_to_record=topic_to_record
        self.fps=fps
        self.codec=codec
        self.depth_range=depth_range
        self.sub=None

        
    def start_recording(self):
        if self.timed_filename:
            self.filename="/tmp/"+datetime.datetime.today().isoformat()+".avi"
        
        args=['rosrun', 
                'image_view', 
                'video_recorder', 
                'image:='+self.topic_to_record, 
                '_filename:='+self.filename, 
                '_max_depth_range:='+str(self.depth_range), 
                '_fps:='+str(self.fps), 
                '_codec:='+self.codec]
        self.p = subprocess.Popen(args,shell=False)
        print args
        print ("\n PID: " + str(self.p.pid))
        #self.p.communicate()
        #self.p = subprocess.call(args,shell=False)
        return args
    
    def stop_recording(self):
        #self.p.send_signal(signal.CTRL_C_EVENT)

        #Kill the process, but write video file
        
        self.p.send_signal(signal.SIGINT)
        
        return self.filename
    
class Depth_and_RGB_Observer_Recorder:
    def __init__(self,topics_to_record,fps=30,codec="X264"):
        self.hd_topic,self.rgb_topic,self.depth_topic=topics_to_record
        self.osr_hd = Observer_Recorder(self.hd_topic,fps=fps,codec=codec)
        self.osr_rgb = Observer_Recorder(self.rgb_topic,fps=fps,codec=codec)
        self.osr_depth = Observer_Recorder(self.depth_topic,fps=fps,depth_range=180)
        
        
    def start_recording(self):
        
        self.osr_hd.start_recording()
        self.osr_rgb.start_recording()
        self.osr_depth.start_recording()
        return True
    
    def stop_recording(self):
        
        file_hd=self.osr_hd.stop_recording()
        file_rgb=self.osr_rgb.stop_recording()
        file_depth=self.osr_depth.stop_recording()
        return (file_hd,file_rgb,file_depth)
    
if __name__ == '__main__':
    #osr=Observer_Recorder("/camera/color/image_raw")
    #osr.start_recording()
    #import time
    #time.sleep(3)
    #print(osr.stop_recording())
    #raw_input()
    #time.sleep(3)
    #osr.start_recording()
    #time.sleep(3)
    #osr.stop_recording()
    topics=("/usb_cam/image_raw","/camera/color/image_raw","/camera/depth/image_raw")
    d_and_rgb_osr=Depth_and_RGB_Observer_Recorder(topics)
    d_and_rgb_osr.start_recording()
    import time
    time.sleep(10)
    (f_hd,f_rgb,f_depth)=d_and_rgb_osr.stop_recording()
    print "Files: " + str((f_hd,f_rgb,f_depth))
    time.sleep(1)
    #import os
    #os.rename(f_rgb,"/tmp/rgb.avi")
    #print ("RGB video in /tmp/rgb.avi")
    #os.rename(f_depth,"/tmp/depth.avi")
    #print ("RGB video in /tmp/depth.avi")



    
