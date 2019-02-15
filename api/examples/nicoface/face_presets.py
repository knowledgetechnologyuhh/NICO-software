from time import sleep

from nicoface.FaceExpression import faceExpression

simulation = False
if simulation:
    fe = faceExpression('sim')
else:
    fe = faceExpression()
    # handcoded presets
    # 'happiness','sadness','anger','disgust','surprise','fear','neutral','clear'
    for expression in ('happiness', 'sadness', 'anger', 'disgust', 'surprise',
                       'fear', 'neutral', 'clear'):
        fe.sendFaceExpression(expression)  # only works with a real robot
        sleep(1)

# trained expressions
for expression in ('Angry', 'Happy', 'Neutral', 'Sad', 'Surprise'):
    print(expression)
    fe.sendTrainedFaceExpression(expression)
    sleep(1)
