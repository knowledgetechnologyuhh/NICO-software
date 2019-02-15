#Say text
import sys
import subprocess
#import  pyttsx3
def say(sen):
	import os.path
	fname="./wav_cache/"+sen+".mp3"
	from gtts import gTTS
	
	
	#try:
			
	if not (os.path.isfile(fname)): 
		import urllib2 
		urllib2.urlopen('http://216.58.192.142', timeout=1)
		tts = gTTS(text=sen, lang='en-au', slow=False)
		#tts.save("/tmp/say.mp3")
		tts.save(fname)
	comm = ["mpg123" , fname]
	subprocess.check_call(comm)
		
	#except:
		#Fallback offline tts engine
		#import pyttsx3;
		#engine = pyttsx3.init();
		#engine.say(sen);
		#engine.runAndWait() ;
		#print ("No internet connection")
		
def face(exp):
	comm = "python /export/NICO/NICO-software/api/src/nicoface/scripts/nicoface/FaceExpression.py " + exp
	subprocess.call(comm, shell=True)
	
	
face(sys.argv[1])
say(sys.argv[2])
