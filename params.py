# -*- coding: utf-8 -*-
"""
The parameters
"""
#Resplution of the grid along the x and y dimensions
gridres_x = 1.0#500.0 #mm
gridres_y = 1.0#500.0 #mm

#Size of grid along X
xmin = 0.0
xmax = 10.0

#Size of grid along Y
ymin = 0.0
ymax = 10.0

#Edge costs
cost = 1.0 #cost for four conrners

#weight of heuristic compared to distance from start 
heuristic_weight = 1.0 #Increasing this will make the code go faster but at the cost of optimality also the heusristic nust be good 

#Robot specific parameters
wheel_dia = 0.140
robot_width = 0.320