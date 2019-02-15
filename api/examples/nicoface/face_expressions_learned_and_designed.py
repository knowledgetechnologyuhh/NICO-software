#works only with pillow library ==2.3.0
# pip install pillow==2.3.0

from nicoface.FaceExpression import faceExpression
from time import sleep
from random import choice

simulation = False
if simulation:
    fe=faceExpression('sim')
else:
    fe = faceExpression()
    # handcoded presets
    # 'happiness','sadness','anger','disgust','surprise','fear','neutral','clear'
    #fe.sendFaceExpression("happiness") # only works with a real robot
    sleep(1)

# trained expressions
for expression in (('Angry','anger'), ('Happy','happiness'), ('Neutral','neutral'), ('Sad', 'sadness'), ('Surprise','surprise')):
	texpress,dexpress=expression	  
	print ("Showing "+dexpress)
	key="b"
	face_choice=choice([True,False])
	while(key=="b"):
		if face_choice:
			print("Trained first")
			fe.sendTrainedFaceExpression(texpress)
			raw_input()
			fe.sendFaceExpression(dexpress)
		else:
			print("Designed first")
			fe.sendFaceExpression(dexpress)
			key=raw_input()
			fe.sendTrainedFaceExpression(texpress)
		key=raw_input()
    #sleep(1)
    
