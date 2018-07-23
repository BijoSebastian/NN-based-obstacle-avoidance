# -*- coding: utf-8 -*-
"""
The astar
"""
#Imports
import numpy as np
import params
import time
import matplotlib.pyplot as plt
import matplotlib.cm as cm

#Close previous matplot lib figures
plt.close("all")

#Class definitions
class LLNode:
    # Constructor: 
    def __init__(self, coords = [0.0, 0.0], g = 0.0, h = float("Inf"), parent = None): 

        self.x = coords[0]
        self.y = coords[1]
        self.g = g
        self.h = h
        self.parent = parent

class obsinfo:
    # Constructor: 
    def __init__(self, coords = [0.0, 0.0], denied_actions = []): 

        self.x = coords[0]
        self.y = coords[1]
        self.obs_actionlist = denied_actions
        
#class LL:
#    # Constructor: 
#    def __init__(self, node = None): 
#        self.head = node     
        
#Globals
global gridres_x, gridres_y #Size of the grid along the x and y dimensions
gridres_x = params.gridres_x 
gridres_y = params.gridres_y 

def expander(cur_node, action):
    #The function that expands the cur_node of the robot based on given action
    global gridres_x, gridres_y
   
    new_x = cur_node.x +  action[0]*gridres_x
    new_y = cur_node.y +  action[1]*gridres_y
    
    return [new_x, new_y]

def heuristic(node, goal_node):
    #The function that calculates heuristic 
    #Manhatten is used as heurstic since no diagoanl motion allowed
    
    return (np.abs(node[0] - goal_node.x) + np.abs(node[1] - goal_node.y))

def Astar(start, goal):
    #The A Star function
    
    #Set up the graphs
    start_node = LLNode(start)
    goal_node = LLNode(goal)
    closed_list = []
    obs_list = []
    fringe = [start_node]
    
    #Define possible actions at each node
    delta = [[-1.0,  0.0], #go left
             [ 0.0, -1.0], #go down
             [ 1.0,  0.0], #go right
             [ 0.0,  1.0]] #go up
    
    #Plot
    plt.scatter(start_node.x, start_node.y, c=cm.autumn(0.0))
    
    found = False #Flag set when search is complete
    resign = False #Flag set when nothing left in fringe
    
    while (found != True and resign != True):

        cur_node = fringe.pop() #POP the least cost node from fringe  
        print('current',cur_node.x, cur_node.y)   
        
        if heuristic([cur_node.x, cur_node.y], goal_node) == 0.0 : #check if we just popped goal if so end now
            print('Found path')
            found = True
            continue
        
        #Add the popped node into closed_list
        closed_list.append(cur_node)

        #Check for allowed actions
        denied_actions = [temp.obs_actionlist for temp in obs_list if [cur_node.x, cur_node.y] == [temp.x, temp.y]]
            
        for i,action in enumerate(delta): 
            
            if i in denied_actions: #apply allowed actions on current state
                continue
            
            new = expander(cur_node, action) #get new node

            if any(new == [temp.x, temp.y] for temp in closed_list): #check if in closed list
                continue
            
            #calculate cost to get to the node 
            new_orient = np.arctan2((new[1] - cur_node.y),(new[0] - cur_node.x)) 
            if cur_node.parent == None:
                old_orient = 0.0                
            else:
                old_orient = np.arctan2((cur_node.y - cur_node.parent.y),(cur_node.x - cur_node.parent.x))
            #Penalise turns to avoid zig zag motion
            if np.abs(old_orient - new_orient) > 3.2:
                cost = params.cost + 1.57
            else:
                cost = params.cost + np.abs(old_orient - new_orient)
            
            
            #Check if we have already visited the node
            visited = False
            for temp in fringe:
                if (new == [temp.x, temp.y]):
                    visited = True
                    if (temp.g < cur_node.g + cost):
                        temp.g = cur_node.g + cost
                        temp.parent = cur_node
            
            #If not add to fringe
            if visited == False:
                new_g = cur_node.g + cost
                new_h = heuristic(new, goal_node)                
                new_node =  LLNode(new, new_g, new_h, cur_node)
                fringe.append(new_node)
            
        if len(fringe) == 0: #Check if any left to expand in fringe if not end now
            print('Failed to find path')
            resign = True
        else:
            fringe  = sorted(fringe, key=lambda llnode: llnode.g+params.heuristic_weight*llnode.h, reverse = True) #sort fringe by g+heusristic
            
            #Debug
            for temp in fringe:
                #print(temp.x, temp.y, temp.g,temp.h ) 
                plt.scatter(temp.x, temp.y, c = cm.autumn((temp.g)/(temp.g+temp.h)))
            #time.sleep(0.1)    
            #input('h')
            
    if found:
        while cur_node.parent != None:
            print(cur_node.x, cur_node.y)
            plt.plot([cur_node.x, cur_node.parent.x], [cur_node.y, cur_node.parent.y], c = [0.0, 0.0, 0.5]);
            cur_node = cur_node.parent            
        print(cur_node.x, cur_node.y)    
        plt.show()    
    
#Testing
start = [0.0, 0.0]
goal = [5.0, 5.0]
Astar(start, goal)    
    
    
    
    
    
