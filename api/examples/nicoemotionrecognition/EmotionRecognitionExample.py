import time
from nicoemotionrecognition import EmotionRecognition

emotionRecogniton = EmotionRecognition.EmotionRecognition()
emotionRecogniton.start(showGUI=True)
time.sleep(10)
print emotionRecogniton.getDimensionalData()
print emotionRecogniton.getCategoricalData()

emotionRecogniton.stop()
