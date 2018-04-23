from nicoface.FaceExpression import faceExpression
from time import sleep

simulation = False
if simulation:
    fe=faceExpression('sim')
else:
    fe = faceExpression()
    # handcoded presets
    # 'happiness','sadness','anger','disgust','surprise','fear','neutral','clear'
    fe.sendFaceExpression("happiness") # only works with a real robot
    sleep(1)

# trained expressions
for expression in ('Angry', 'Happy', 'Neutral', 'Sad', 'Surprise'):
    fe.sendTrainedFaceExpression(expression)
    sleep(1)
