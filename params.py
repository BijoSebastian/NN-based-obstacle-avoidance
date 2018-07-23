# -*- coding: utf-8 -*-
"""
The parameters
"""
#Size of the grid along the x and y dimensions
gridres_x = 1.0#500.0 #mm
gridres_y = 1.0#500.0 #mm

#Edge costs
cost = 1.0 #cost for four conrners

#weight of heuristic compared to distance from start 
heuristic_weight = 1.0 #Increasing this will make the code go faster but at the cost of optimality also the heusristic nust be good 
