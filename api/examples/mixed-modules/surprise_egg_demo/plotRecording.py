from random import random
import numpy as np
import time
import matplotlib.pyplot as plt
import os
import wave

plt.ion()
#fig = plt.figure()
#fig.subplots_adjust(top=0.8)
f, axarr = plt.subplots(2, sharex=True)

axarr[0].set_title('Seconds')
axarr[0].set_ylabel('left channel')
axarr[0].set_title('level')
#ax2 = fig.add_subplot(211)
axarr[1].set_ylabel('right channel')
axarr[0].set_autoscale_on(False)
axarr[1].set_autoscale_on(False)
#ax2.set_ylabel('time (s)')
#n, bins, patches = ax2.hist(np.random.randn(1000), 50,
#facecolor='yellow', edgecolor='yellow')
#t = np.arange(0.0, 1.0, 0.01)
#s = np.sin(random()*2*np.pi*t)
#line, = ax1.plot(t, s, color='blue', lw=2)
#fig.canvas.draw()
#fig.canvas.flush_events()
#time.sleep(4)


for x in range(36000):
    
    #check if file has changed
    if (True):
        filename="./sub_.wav"
        #filename="./speech.wav"
        print 'Splitting files'
        f1 = "recording.l.wav"
        os.system("sox " + filename + " " + f1 + " remix 1")
        f1 = "recording.r.wav"
        os.system("sox " + filename + " " + f1 + " remix 2")
        
        #leftchannel
        try:
            print "Reading recording.l.wav"
            spf = wave.open("recording.l.wav",'r')
        except:
            continue
            
        #Extract Raw Audio from Wav File
        signal = spf.readframes(-1)
        signal = np.fromstring(signal, 'Int16')
        fs = spf.getframerate()
        print "Framerate" + str(fs) +" Length: " + str(len(signal)/fs) + " Number " + str (len(signal))
        Time=np.linspace(0, len(signal)/float(fs), num=len(signal))
        print str (Time)
        
        #self.linesl.set_xdata(Time)
        #Time = np.arange(0.0, 1.0, 0.01)
        #signal = np.sin(random()*2*np.pi*t)
        axarr[0].clear()
        line, = axarr[0].plot(Time,signal,color='blue')
        axarr[0].axis([0.0,0.7,-40000.0,+40000.0])
        #self.linesl.set_ydata(signal)
        #plt.show()
        #Need both of these in order to rescale
        #axarr[0].relim()
        #axarr[0].autoscale_view()
        
        #rightchannel
        try:
            print "Reading recording.l.wav"
            spf = wave.open("recording.r.wav",'r')
        except:
            continue
            
        #Extract Raw Audio from Wav File
        signal = spf.readframes(-1)
        signal = np.fromstring(signal, 'Int16')
        fs = spf.getframerate()
        print "Framerate" + str(fs) +" Length: " + str(len(signal)/fs) + " Number " + str (len(signal))
        Time=np.linspace(0, len(signal)/float(fs), num=len(signal))
        print str (Time)
        
        #self.linesl.set_xdata(Time)
        #Time = np.arange(0.0, 1.0, 0.01)
        #signal = np.sin(random()*2*np.pi*t)
        axarr[1].clear()
        line, = axarr[1].plot(Time,signal,color='blue')
        axarr[1].axis([0.0,0.7,-40000.0,+40000.0])
        #self.linesl.set_ydata(signal)
        #plt.show()
        #Need both of these in order to rescale
        #axarr[1].relim()
        #axarr[1].autoscale_view()
        
        f.canvas.draw()
        #plt.pause(0.001)
        f.canvas.flush_events()
        time.sleep(0.2)
        

    


