# -*- coding: utf-8 -*-
"""
The parameters
"""
import numpy as np

#Resplution of the grid along the x and y dimensions
gridres_x = 500.0 #mm
gridres_y = 500.0 #mm

#Size of grid along X
xmin = 0.0
xmax = 2500#10.0*gridres_x

#Size of grid along Y
ymin = 0.0
ymax = 3000#10.0*gridres_y

#weight of heuristic compared to distance from start 
heuristic_weight = 1.0 #Increasing this will make the code go faster but at the cost of optimality also the heusristic nust be good 

#Robot specific parameters
wheel_dia = 0.140
robot_width = 0.320

#Global goal
x_g = 1500.0 #mm
y_g = 2500.0 #mm

#Controller parameters
#Gains
Kp = 0.156
Kd = 0.0002
Ki = 0.0
#Previous error history
total_heading_error = 0.0
prev_heading_error = 0.0

#Threshold distance
#1byfourth of the diagoanl grid resolution
thresh_dist = 100.0#(np.sqrt((gridres_x**2) + (gridres_y**2)))/10.0

#Permenant obstacle info assuming the world do not change
class obsinfo:
    # Constructor: 
    def __init__(self, coords = [0.0, 0.0], denied_actions = []): 

        self.x = coords[0]
        self.y = coords[1]       
        self.obs_actionlist = denied_actions

#global obs_list
obs_list = []

#Define possible actions at each node
delta = [[-1.0,  0.0], #go left
         [ 0.0, -1.0], #go down
         [ 1.0,  0.0], #go right
         [ 0.0,  1.0]] #go up

#CNN obstacle count for detection
cnn_count = 10

#Data recording file
filename = open("pose.txt", "w")
filename.write("start\n")
#Closed at the end of main