import time
from nicoemotionrecognition import EmotionRecognition

emotionRecogniton = EmotionRecognition.EmotionRecognition()
emotionRecogniton.start(showGUI=True, mirrorEmotion=True)
raw_input("Press enter to stop\n")
print emotionRecogniton.getDimensionalData()
print emotionRecogniton.getCategoricalData()

emotionRecogniton.stop()
