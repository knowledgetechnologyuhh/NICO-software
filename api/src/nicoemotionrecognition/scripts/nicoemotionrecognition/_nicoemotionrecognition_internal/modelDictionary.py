# encoding=utf8
import os.path as path
import sys

if sys.version_info < (3,):
    reload(sys)
    sys.setdefaultencoding("utf8")


class CategoricaModel:
    modelname = "Categorical_FER 2013 Plus"
    modelDirectory = (
        path.dirname(path.abspath(__file__))
        + "/Trained Networks/Vision/FER2013Plus_Augmented_CNN/Model/CNN.h5"
    )
    if sys.version_info >= (3,):
        modelDirectory = modelDirectory.replace("CNN.h5", "CNN-python3.h5")
    modelType = "Categorical"
    classsesOrder = [
        "Neutral",
        "Happiness",
        "Surprise",
        "Sadness",
        "Anger",
        "Disgust",
        "Fear",
        "Contempt",
    ]
    classesColor = [
        (255, 255, 255),
        (0, 255, 0),
        (0, 222, 255),
        (255, 0, 0),
        (0, 0, 255),
        (255, 0, 144),
        (0, 144, 255),
        (75, 75, 96),
    ]


class DimensionalModel:
    modelname = "Arousal and Valence TrainedOnAffew"
    modelDirectory = (
        path.dirname(path.abspath(__file__))
        + "/Trained Networks/Vision/AFFEW_Vision_ShallowNetwork/Model/weights.best.hdf5"
    )
    modelType = "Dimensional"
    classsesOrder = ["Arousal", "Valence"]
    classesColor = [(0, 255, 0), (255, 0, 0)]

    # ["'0':'Neutral',", "'1':'Surprise',", "'2':'Sad',", "'3':'Disgust',", "'4':'Angry',", "'5':'Fear',", "'6':'Happy',"]
