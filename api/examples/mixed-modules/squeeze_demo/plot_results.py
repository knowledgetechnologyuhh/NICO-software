from random import random
import numpy as np
import time
import matplotlib.pyplot as plt
import os

label = ["red_tomato", "green\ncucumber", "red_ball", "yellow\nbanana",
           "red_banana", "yellow_dice", "green_pepper", "blue_ball",
           "red_dice", "purple\ngrapes", "red_sponge", "orange\ncarrot",
           "black_hat", "purple_duck", "orange_fish", "green_figure"]

data = [  1.46257728e-09  , 1.38053175e-16  , 3.80929521e-07 ,  3.66634558e-16,
    2.00181369e-18  , 2.93641780e-02 ,  4.43123270e-07 ,  1.81522676e-18,
    3.31479329e-17 ,  4.25515883e-03  , 8.02651470e-14 ,  5.79644510e-09,
    4.40786340e-12  , 1.00490233e-05 ,  1.93719563e-22 ,  9.66369808e-01]

plt.ion()
plt.rcParams['toolbar'] = 'None'
#fig = plt.figure()
#fig.subplots_adjust(top=0.8)
f, axarr = plt.subplots(1, sharex=True)

axarr.set_title('Network results (probabilities)')
index = np.arange(len(label))+0.5
#plt.yticks(index, label, fontsize=15, rotation=30)
#axarr[0].set_ylabel('left channel')
#axarr[0].set_title('level')
#ax2 = fig.add_subplot(211)
#axarr[1].set_ylabel('right channel')
##axarr[0].set_autoscale_on(False)
##axarr[1].set_autoscale_on(False)
#ax2.set_ylabel('time (s)')
#n, bins, patches = ax2.hist(np.random.randn(1000), 50,
#facecolor='yellow', edgecolor='yellow')
#t = np.arange(0.0, 1.0, 0.01)
#s = np.sin(random()*2*np.pi*t)
#line, = ax1.plot(t, s, color='blue', lw=2)
#fig.canvas.draw()
#fig.canvas.flush_events()
#time.sleep(4)

data_filename="/tmp/all_results.csv"
last_change=None

for x in range(36000):
    
    #check if file has changed
    
    if os.path.isfile(data_filename):
        curr_change=os.stat(data_filename).st_mtime
        if (curr_change!=last_change):

            last_change=curr_change

            data = np.loadtxt(data_filename)
            
            axarr.clear()
            plt.yticks(index, label, fontsize=16, rotation=0)
            
            
            axarr.barh(index, data,align='center')
            
            #self.linesl.set_xdata(Time)
            #Time = np.arange(0.0, 1.0, 0.01)
            #signal = np.sin(random()*2*np.pi*t)
            #axarr[0].clear()
            #line, = axarr[0].plot(Time,signal,color='blue')
            #axarr[0].axis([0.0,0.7,-40000.0,+40000.0])
            #self.linesl.set_ydata(signal)
            #plt.show()
            #Need both of these in order to rescale
            #axarr[0].relim()
            #axarr[0].autoscale_view()
            
            #rightchannel
            #try:
            #    print "Reading recording.l.wav"
            #    spf = wave.open("recording.r.wav",'r')
            #except:
            #    continue
                
            #axarr[1].clear()
            #line, = axarr[1].plot(Time,signal,color='blue')
            #axarr[1].axis([0.0,0.7,-40000.0,+40000.0])
            
            #self.linesl.set_ydata(signal)
            #plt.show()
            #Need both of these in order to rescale
            #axarr[1].relim()
            #axarr[1].autoscale_view()
            
            f.canvas.draw()
            #plt.pause(0.001)
            f.canvas.flush_events()
            import time
            timestr = time.strftime("%Y%m%d-%H%M%S")
            f.savefig('./'+timestr+'.png')
    time.sleep(1)
        

    


