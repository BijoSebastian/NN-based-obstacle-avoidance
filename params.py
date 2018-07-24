# -*- coding: utf-8 -*-
"""
The parameters
"""
#Resplution of the grid along the x and y dimensions
gridres_x = 500.0 #mm
gridres_y = 500.0 #mm

#Size of grid along X
xmin = 0.0
xmax = 10.0*gridres_x

#Size of grid along Y
ymin = 0.0
ymax = 10.0*gridres_y

#Edge costs
cost = 1.0 #cost for four conrners

#weight of heuristic compared to distance from start 
heuristic_weight = 1.0 #Increasing this will make the code go faster but at the cost of optimality also the heusristic nust be good 

#Robot specific parameters
wheel_dia = 0.140
robot_width = 0.320

#Global goal
x_g = 4000.0 #mm
y_g = 4000.0 #mm

#Controller parameters
#Gains
Kp = 0.656
Kd = 0.0002
Ki = 0.0
#Previous error history
total_heading_error = 0.0
prev_heading_error = 0.0

#Permenant obstacle info assuming the world do not change
class obsinfo:
    # Constructor: 
    def __init__(self, coords = [0.0, 0.0], denied_actions = []): 

        self.x = coords[0]
        self.y = coords[1]       
        self.obs_actionlist = denied_actions

#global obs_list
obs_list = []