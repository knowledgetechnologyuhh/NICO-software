# -*- coding: utf-8 -*-
"""
Created on Tue Jul  3 16:50:12 2018

@author: twiefel(test_client), strahl
"""

def observer_client_wrapper():
    from observer_recorder import Depth_and_RGB_Observer_Recorder
    

    #create the observer_recorder object 
    topics=("/usb_cam/image_raw","/camera/color/image_raw","/camera/depth/image_raw")
    d_and_rgb_osr=Depth_and_RGB_Observer_Recorder(topics)

    #and to some attribute and method calls
    #d_and_rgb_osr.start_recording()
    #import time
    #time.sleep(3)
    #(f_rgb,f_depth)=d_and_rgb_osr.stop_recording()
    #import os
    #os.rename(f_rgb,"/tmp/rgb.avi")
    #print ("RGB video in /tmp/rgb.avi")
    #os.rename(f_depth,"/tmp/depth.avi")
    #print ("RGB video in /tmp/depth.avi")
    
    from client import ClientWrapper

    #we create a test object by wrapping the TestClass in a client Wrapper
    #only the port has to be specified.
    #if the server runs on a server, you can specify the address like his:
    #   test_object = ClientWrapper(TestClass, 54010, server='wtmgws9')   
    d_and_rgb_osr = ClientWrapper(Depth_and_RGB_Observer_Recorder, 54010, server='wtmpc211')
    
    #we can do the same manipulations here. 
    #at the moment, only types that can be wrapped in json can be used
    #as arguments or return value
    #if you use non jsonifiable types, the server or client will throw a json exception
    
    d_and_rgb_osr.start_recording()
    import time
    time.sleep(10)
    (f_hd,f_rgb,f_depth)=d_and_rgb_osr.stop_recording()
    #import os
    #os.rename(f_rgb,"/tmp/rgb_remote.avi")
    #print ("RGB video in /tmp/rgb_remote.avi")
    #os.rename(f_depth,"/tmp/depth_remote.avi")
    #print ("RGB video in /tmp/depth_remote.avi")
    from sc_copy import sc_copy
    import os
    sc_copy("wtmpc211:"+f_hd,"/tmp/hd_"+os.path.basename(f_hd))
    sc_copy("wtmpc211:"+f_rgb,"/tmp/rgb_"+os.path.basename(f_rgb))
    sc_copy("wtmpc211:"+f_depth,"/tmp/depth_"+os.path.basename(f_depth))
    

def main():
    observer_client_wrapper()
    
if __name__ == "__main__":
    main()
