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
        
#Globals
global obs_list
obs_list = []

def boundary_builder():
    global obs_list
    
    for i in np.arange(params.ymin + params.gridres_y , params.ymax, params.gridres_y):
        temp = obsinfo([params.xmin, i], [0])
        obs_list.append(temp)
        
    for i in np.arange(params.ymin + params.gridres_y , params.ymax, params.gridres_y):
        temp = obsinfo([params.xmax, i], [2])
        obs_list.append(temp)
        
    for i in np.arange(params.xmin + params.gridres_x , params.xmax, params.gridres_x):
        temp = obsinfo([i, params.ymin], [1])
        obs_list.append(temp)
        
    for i in np.arange(params.xmin + params.gridres_x , params.xmax, params.gridres_x):
        temp = obsinfo([i, params.ymax], [3])
        obs_list.append(temp) 
    
    temp = obsinfo([params.xmin, params.ymin], [0,1])
    obs_list.append(temp)
    temp = obsinfo([params.xmax, params.ymin], [2,1])
    obs_list.append(temp)
    temp = obsinfo([params.xmin, params.ymax], [0,3])
    obs_list.append(temp)
    temp = obsinfo([params.xmax, params.ymax], [2,3])
    obs_list.append(temp)
     
    #Debug
    #for i in range(len(obs_list)):
        #print(obs_list[i].x, obs_list[i].y, obs_list[i].obs_actionlist)
        
def expander(cur_node, action):
    #The function that expands the cur_node of the robot based on given action
   
    new_x = cur_node.x +  action[0]*params.gridres_x
    new_y = cur_node.y +  action[1]*params.gridres_y 
    
    return [new_x, new_y]

def heuristic(state, goal):
    #The function that calculates heuristic 
    #Manhatten is used as heurstic since no diagoanl motion allowed
    
    #return np.sqrt((state[0] - goal[0])**2 + (state[1] - goal[1])**2)
    return (np.abs(state[0] - round(goal[0])) + np.abs(state[1] - round(goal[1])))

def at_goal(state, goal):
    #Function to reach if at goal
    #Using half diagoal length of cell as threshold distance to decide if goal is nearby
    thresh_dist = (np.sqrt((params.gridres_x**2) + (params.gridres_y**2)))/2.0
    
    dist = np.sqrt((state[0] - goal[0])**2 + (state[1] - goal[1])**2)
    
    if dist <= thresh_dist:
        return True
    else:
        return False

def Astar(start, goal):
    #The A Star function
    
    #Set up the graphs
    start_node = LLNode(start)
    goal_node = LLNode(goal)
    closed_list = []
    global obs_list
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
        
        #check if we just popped goal if so end now
        if at_goal([cur_node.x, cur_node.y], [goal_node.x, goal_node.y]): 
            print('Found path')
            found = True
            continue
        
        #Add the popped node into closed_list
        closed_list.append(cur_node)

        #Check for allowed actions
        denied_actions = [temp.obs_actionlist for temp in obs_list if [cur_node.x, cur_node.y] == [temp.x, temp.y]]
        if len(denied_actions) > 0:
            denied_actions = denied_actions[0]
        #print(denied_actions, )    
        
        for i,action in enumerate(delta): 
            
            if i in denied_actions: #apply allowed actions on current state
                #print('bad action', i)
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
                cost = params.cost + 1.58
            else:
                cost = params.cost + np.abs(old_orient - new_orient)
            
            
            #Check if we have already visited the node
            visited = False
            for temp in fringe:
                if (new == [temp.x, temp.y]):
                    visited = True
                    if (temp.g > cur_node.g + cost):
                        temp.g = cur_node.g + cost
                        temp.parent = cur_node
            
            #If not add to fringe
            if visited == False:
                new_g = cur_node.g + cost
                new_h = heuristic(new, [goal_node.x, goal_node.y])                
                new_node =  LLNode(new, new_g, new_h, cur_node)
                fringe.append(new_node)
            
        if len(fringe) == 0: #Check if any left to expand in fringe if not end now
            print('Failed to find path')
            resign = True
        else:
            fringe  = sorted(fringe, key=lambda llnode: llnode.g+(params.heuristic_weight*llnode.h), reverse = True) #sort fringe by g+heusristic
            
            #Debug
            for temp in fringe:
                print(temp.x, temp.y, temp.g,temp.h ) 
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
    
# =============================================================================
# #Testing
# start = [0.0, 0.0]
# goal = [7.6, 8.4]
# boundary_builder()
# Astar(start, goal)    
# =============================================================================
    
    
    
    
    
