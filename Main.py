# -*- coding: utf-8 -*-
"""
The main file
"""
#import libraries
import time
import numpy as np

#Import files
import params
import CNN
import Planner
import Controller

########################Utility################################################
def look_around(vrep, clientID, kinectd_h, robot_Handle, robot_LeftMotorHandle, robot_RightMotorHandle):
    #function to gather obstacle info
    denied_actions = []
    flag = False
   
    sides = [3.14, -1.57, 0.0, 1.57]
    #Cycle through all four sides and see which actions are possible and which not
    for i in range(4):
        
        #Allign the robot
        state = Controller.localise(vrep, clientID, robot_Handle)
        while abs(state[2] - sides[i]) > 0.1:
            [V,W] = Controller.orient_robot(state[2], sides[i])
            Controller.robot_setvel(V,W, vrep, clientID, robot_LeftMotorHandle, robot_RightMotorHandle)
            state = Controller.localise(vrep, clientID, robot_Handle)  
            time.sleep(0.5)
        params.total_heading_error = 0.0
        params.prev_heading_error = 0.0
        time.sleep(2)

        #Get the sensor info
        res, res1, buffer = vrep.simxGetVisionSensorDepthBuffer(clientID, kinectd_h, vrep.simx_opmode_buffer) 
        len(buffer)
        
        res , robot_Orientation = vrep.simxGetObjectOrientation(clientID, robot_Handle, -1 , vrep.simx_opmode_buffer)
        alpha = np.rad2deg(robot_Orientation[0])
        alpha = (alpha + 40.0)/80.0
        beta =  np.rad2deg(robot_Orientation[1])
        beta = (beta + 40.0)/80.0
        temp = CNN.model.predict([buffer[None,...,None], np.array(float(alpha))[None,...], np.array(float(beta))[None,...]])
        temp = params.obsinfo([params.xmin, params.ymin], [0,1])
        if temp[0][0] == 0:
            flag = True
            denied_actions.append(i)
    
    #Get the grid based location as seen by the planner
    state = Controller.localise(vrep, clientID, robot_Handle)
    state[0]  = (round(state[0]/params.gridres_x))*params.gridres_x
    state[1]  = (round(state[0]/params.gridres_y))*params.gridres_y
    
    flag2 = False
    #If atleast one denied action    
    if flag:
        #If already existing then just update
        for node in params.obs_list:
            if [node.x, node.y] == state[0:2]:
                node.obs_actionlist = denied_actions
                flag2 = True
                break
            
        #If not existing then just append    
        if flag2 == False:
            temp = params.obsinfo(state[0:2],denied_actions)
            params.obs_list.append(temp)               
    
###############################Main############################################
try:    
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

#Setup the simulation
vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19997,True,True,2000,5) # Connect to V-REP

if clientID!=-1:

    print ('Connected to remote API server')
    
    ###Get a handles to the robot:    
    res , robot_Handle = vrep.simxGetObjectHandle(clientID, "HMMR", vrep.simx_opmode_blocking)
    # Get the position of the Pioneer for the first time in streaming mode
    res , robot_Position = vrep.simxGetObjectPosition(clientID, robot_Handle, -1 , vrep.simx_opmode_streaming)
    res , robot_Orientation = vrep.simxGetObjectOrientation(clientID, robot_Handle, -1 , vrep.simx_opmode_streaming)
    
    robot_LeftMotorHandle = []
    robot_RightMotorHandle = []
    for i in range(4):
        #Left
        res,  temp = vrep.simxGetObjectHandle(clientID, "LeftMotor"+ str(i), vrep.simx_opmode_blocking)
        robot_LeftMotorHandle.append(temp)
        res = vrep.simxSetJointTargetVelocity(clientID, temp, 0.0, vrep.simx_opmode_streaming)
        
        #Right
        res,  temp = vrep.simxGetObjectHandle(clientID, "RightMotor"+ str(i), vrep.simx_opmode_blocking)
        robot_RightMotorHandle.append(temp)
        res = vrep.simxSetJointTargetVelocity(clientID, temp, 0.0, vrep.simx_opmode_streaming)
    
    #Get kinect handle
    res, kinectd_h = vrep.simxGetObjectHandle(clientID,"kinect_depth",vrep.simx_opmode_blocking)
    res, res1, buffer = vrep.simxGetVisionSensorDepthBuffer(clientID, kinectd_h, vrep.simx_opmode_streaming)            
        
else:
    print('Failed connecting to remote API server!!')
    ######################################################################
    
    #Drop the robot at the desired start location with the correct orientation before starting the simulation        
    
    #Get all the sensor reading and scale them as required
    
    #The procedure drop robot take time to stabilise turn stabilise record roll pitc and kinect  then turn and do same in all directions pass data to cnn and get obstacle map then pass data to a start get decision and tehn move forward.
    ######################################################################

#Position the robot
vrep.simxSetObjectPosition(clientID, robot_Handle, -1, [0.0, 0.0, 0.0], vrep.simx_opmode_oneshot)
vrep.simxSetObjectOrientation(clientID, robot_Handle, -1, [0.0, 0.0, 0.0], vrep.simx_opmode_oneshot)
time.sleep(4)#So that the motion is completed
    
#Start the simulation
res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
if res == vrep.simx_return_ok:
    print ("---!!! Started Simulation !!! ---")
    time.sleep(4)#So that the robot stabilizes

#Extract current location of the robot 
state = Controller.localise(vrep, clientID, robot_Handle)

#Define main gaol
main_goal = [params.x_g, params.y_g]

#start the run
while Planner.at_goal(state[0:2], main_goal) == False:
    
    #Gather information and store obstacle info in global thingy
    look_around(vrep, clientID, kinectd_h, robot_Handle, robot_LeftMotorHandle, robot_RightMotorHandle)
    
    
    
