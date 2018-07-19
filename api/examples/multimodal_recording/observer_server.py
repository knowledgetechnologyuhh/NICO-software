"""
Created on Tue Jul  3 16:50:04 2018

@author: twiefel (test_server_wrapper), strahl
"""

def recorder_server_wrapper():
    from server import ServerWrapper
    from observer_recorder import Depth_and_RGB_Observer_Recorder
    
    #create the observer_recorder object 
    topics=("/camera/rgb/image_raw","/camera/depth/image_raw")
    d_and_rgb_osr=Depth_and_RGB_Observer_Recorder(topics)

    #wrap the test object in the server wrapper.
    #e voila, the test object can be used by the client
    d_and_rgb_osr = ServerWrapper(d_and_rgb_osr,54010)
    
def main():
    recorder_server_wrapper()
    
if __name__ == "__main__":
    main()
