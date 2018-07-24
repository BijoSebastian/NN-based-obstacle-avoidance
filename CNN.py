# -*- coding: utf-8 -*-
"""
Hailins code modified for use.
"""
import numpy as np
import cv2
import os
import os.path
import struct
import h5py
import csv
import pylab as plt

from keras.models import Model
from keras.layers.merge import Concatenate
from keras.layers import Activation, Input, Lambda, Dense, Flatten, Dropout
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.merge import Multiply
from keras.regularizers import l2
from keras.initializers import random_normal,constant

def get_testing_model():
    #function to get....
    
    img_input_shape = (48, 64,1)
    IMU1_input_shape = (1,)
    IMU2_input_shape = (1,)

    inputs = []
    outputs = []

    img_input = Input(shape=img_input_shape)
    IMU1_input = Input(shape=IMU1_input_shape)
    IMU2_input = Input(shape=IMU2_input_shape)

    inputs.append(img_input)
    inputs.append(IMU1_input)
    inputs.append(IMU2_input)
    
    x = Conv2D(32, (2, 2), activation = 'relu')(img_input)
    x = MaxPooling2D(pool_size =(2, 2))(x)
    x = Conv2D(32, (2, 2), activation = 'relu')(x)
    x = MaxPooling2D(pool_size =(2, 2))(x)
    x = Flatten()(x)

    merged = Concatenate()([x, IMU1_input, IMU2_input])
    x = Dense(256, activation='relu')(merged)
    x = Dense(256, activation='relu')(x)
    predictions = Dense(1, activation='sigmoid')(x)
    

    model = Model(inputs=inputs, outputs=predictions)
    return model

weights_path = "weights.best.h5"
model = get_testing_model()
model.load_weights(weights_path)

#All of the above sould happen before everything starts up
#The next one line is for the testing 
model.predict([img[None,...,None], np.array(float(IMU1))[None,...], np.array(float(IMU2))[None,...]])