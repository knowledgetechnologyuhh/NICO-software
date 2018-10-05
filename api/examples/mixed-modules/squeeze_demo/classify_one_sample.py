# -*- coding: utf-8 -*-
"""
Created on Wed Nov  1 15:15:36 2017

@author: strahl, kerzel
"""

from __future__ import print_function
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras import backend as K
from keras.models import load_model

#Example for getting data from Experiment data
import sqlite3
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.fftpack import fft, ifft
import math

import sys

import os
from os import listdir
from os.path import isfile, join



def normalize(number, minimum, maximum):
    if maximum != minimum:
        return ((float(number)-float(minimum)) / (float(maximum)-float(minimum)))
    else:
        return 0

def numberToString(number):
    table = []
    table.append("red_tomato")
    table.append("green_sausage")
    table.append("red_ball")
    table.append("yellow_banana")    
    table.append("red_banana")
    table.append("yellow_dice")
    table.append("green_pepper")
    table.append("blue_ball")    
    table.append("red_dice")
    table.append("puple_grapes")
    table.append("red_sponge")
    table.append("orange_carrot")    
    table.append("black_hat")
    table.append("purple_duck")
    table.append("orange_fish")
    table.append("green_figure")        
    return table[number]

def classify(filename):   

    connection = sqlite3.connect(filename)
    cursor = connection.cursor()

    x_all = []
    y_all = []

    sample_matrix = []
            
    cursor.execute("SELECT * FROM sample,subsample where sample.sample_number=subsample.sample_number")
    #cursor.execute("SELECT * FROM sample,subsample where sample.sample_number=subsample.sample_number \
    #and sample.object_name='" + numberToString(item) +"' and subsample.sample_number=" +  str(i))

    result = cursor.fetchall()

    sample_series = []

    if result != []:

        for r in result:

            print("DEBUG: result")
            print(r)
            #raw_input()
            sample = []
            #data_entry = r.split()
            split_entry = str(r).split(',')
        
            """
            [4] sample series
            
            [6] = thumb_position
            [7] = finger_position
            [8] = thumb_current
            [9] = finger_current
            
            [14] = x_touch
            [15] = y_touch
            [16] = z_touch
            """
            
            sample.append(float(split_entry[6]))
            sample.append(float(split_entry[7]))
            sample.append(float(split_entry[8]))
            sample.append(float(split_entry[9]))

        
            sample.append(float(split_entry[14]))
            sample.append(float(split_entry[15]))
            sample.append(float(split_entry[16]))

        
            #compute L2 norm for force vector
            l2_norm = math.sqrt(
                    math.pow(float(split_entry[14]),2)+
                    math.pow(float(split_entry[15]),2)+
                    math.pow(float(split_entry[16]),2))
        
            sample.append(l2_norm)
                    
            sample_series.append(sample)
            
        sample_matrix.append(sample_series)
        x_all.append(sample_series)            
        y_all.append(1)
    else:
        print("result empty!")
        
    #create the x and y arrays for neural learning
    x_all = np.array(x_all)
    y_all = np.array(y_all)

    print("DEBUG x_all")
    print(x_all)

    #normalize the values
    [a1,a2,a3,a4,a5,a6,a7,a8] = np.dsplit(x_all,[1,2,3,4,5,6,7])    

    norm_array=[-180.0,-180.0, 6.0, 6.0, -354.0, -574.0, -493.0, 14.45683229480096, 63.69, 5580.88, 143.0, 216.0, 975.0, 375.0, 39.0, 1021.5698703466151]    

        
    #change to make sure the values are capped at 0 and 1
    for i in range(len(x_all)):
        for j in range(len(x_all[0])):
            x_all[i][j][0] = normalize(x_all[i][j][0],norm_array[0],norm_array[8])
            x_all[i][j][1] = normalize(x_all[i][j][1],norm_array[1],norm_array[9])
            x_all[i][j][2] = normalize(x_all[i][j][2],norm_array[2],norm_array[10])
            x_all[i][j][3] = normalize(x_all[i][j][3],norm_array[3],norm_array[11])
            x_all[i][j][4] = normalize(x_all[i][j][4],norm_array[4],norm_array[12])
            x_all[i][j][5] = normalize(x_all[i][j][5],norm_array[5],norm_array[13])
            x_all[i][j][6] = normalize(x_all[i][j][6],norm_array[6],norm_array[14])
            x_all[i][j][7] = normalize(x_all[i][j][7],norm_array[7],norm_array[15])


    x_test = np.array(x_all)

    x_test = x_test.astype('float32')
    
    x_test = np.reshape(x_test, (len(x_test),52,8))
        
    ######################## BEGIN NEURAL NETWORK CLASSIFICATION ##########

    print("Loading neural network")

    num_classes = 16

    img_rows, img_cols = 52, 8

    x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols, 1)
    input_shape = (img_rows, img_cols, 1)

    #model = Sequential()
    #model.add(Conv2D(64, kernel_size=(6, 8),
    #                 activation='relu',
    #                 input_shape=input_shape))
                    
    #model.add(Flatten())
    #model.add(Dense(100, activation='relu'))

    #model.add(Dense(num_classes, activation='softmax'))

    print("loading model")
    model=keras.models.load_model("./Net5-latest_trained_modell.h5")

    network_input = np.reshape(x_test[i], (1,52,8,1))
        
    classification = model.predict(network_input)

    return(np.argmax(classification))

    #for i in range(len(x_test)):
    #    network_input = np.reshape(x_test[i], (1,52,8,1))
        
    #    classification = model.predict(network_input)

        
    #    print("Item classification= " + str(np.argmax(classification)))
    #    print()


if __name__ == "__main__":

    result=classify("/tmp/one_sample.db")
    print ("This is a " + numberToString(result))
