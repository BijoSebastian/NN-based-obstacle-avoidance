# -*- coding: utf-8 -*-
"""
The main file
"""
#import libraries
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

#Import files
import params
import CNN
import Planner
import Controller       
    
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
        res,  temp = vrep.simxGetObjectHandle(clientID, "LeftMotor"+ str(i+1), vrep.simx_opmode_blocking)
        robot_LeftMotorHandle.append(temp)
        res = vrep.simxSetJointTargetVelocity(clientID, temp, 0.0, vrep.simx_opmode_streaming)
        
        #Right
        res,  temp = vrep.simxGetObjectHandle(clientID, "RightMotor"+ str(i+1), vrep.simx_opmode_blocking)
        robot_RightMotorHandle.append(temp)
        res = vrep.simxSetJointTargetVelocity(clientID, temp, 0.0, vrep.simx_opmode_streaming)
    
    #Get kinect handle
    res, kinectd_h = vrep.simxGetObjectHandle(clientID,"kinect_depth",vrep.simx_opmode_oneshot_wait)
    res, res1, buffer = vrep.simxGetVisionSensorDepthBuffer(clientID, kinectd_h, vrep.simx_opmode_streaming)            

else:
    print('Failed connecting to remote API server!!')

#Position the robot
vrep.simxSetObjectPosition(clientID, robot_Handle, -1, [0.0, 0.0, 0.0], vrep.simx_opmode_oneshot)
vrep.simxSetObjectOrientation(clientID, robot_Handle, -1, [0.0, 0.0, 0.0], vrep.simx_opmode_oneshot)
time.sleep(4)#So that the motion is completed

#Define main gaol
main_goal = [params.x_g, params.y_g]
 
#Build the planning boundaries
Planner.boundary_builder()
    
#Start the simulation
res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
if res == vrep.simx_return_ok:
    print ("---!!! Started Simulation !!! ---")
    time.sleep(4)#So that the robot stabilizes

#Extract current location of the robot 
state = Controller.localise(vrep, clientID, robot_Handle)
  
#start the run
while Planner.at_goal(state[0:2], main_goal) == False:  
    
    #Get the new local goal from the plan
    path, action_list = Planner.Astar(state[0:2], main_goal)
    if path == None:
        break
    else:
        l_goal = path[-2]
        n_action = action_list[-2]
    
    #Set params:
    flag_go = True
    flag_prev = True
    flag_exist = False
    counter = 0
        
    #Go to local goal
    print('Moving to local goal', l_goal)
    while flag_go:
        [V,W] = Controller.gtg(state,l_goal)
        Controller.robot_setvel(V,W, vrep, clientID, robot_LeftMotorHandle, robot_RightMotorHandle)
        state = Controller.localise(vrep, clientID, robot_Handle)              
        #print('state', state)
        
        #Get the sensor info
        res, res1, buffer = vrep.simxGetVisionSensorDepthBuffer(clientID, kinectd_h, vrep.simx_opmode_oneshot_wait)     
        buffer = np.reshape(buffer,(48,64))
        buffer = buffer*255
        buffer = buffer.astype(int)
        buffer = cv2.flip( buffer, 0 )
        res , robot_Orientation = vrep.simxGetObjectOrientation(clientID, robot_Handle, -1 , vrep.simx_opmode_buffer)
        alpha = np.rad2deg(robot_Orientation[0])
        alpha = (alpha + 40.0)/80.0
        beta =  np.rad2deg(robot_Orientation[1])
        beta = (beta + 40.0)/80.0
        temp = CNN.model.predict([buffer[None,...,None], np.array(float(alpha))[None,...], np.array(float(beta))[None,...]])
        #print(temp[0][0])
        if temp[0][0] <= 0.2:         
            if flag_prev == False:
                counter +=1
            flag_prev = False
        else:
            counter = 0
            flag_prev = True
            
        #Checks
        if counter > params.cnn_count or Planner.at_goal(state[0:2], l_goal):
            flag_go = False   
            if Planner.at_goal(state[0:2], l_goal):
                print('reached goal')                
            else:
                print('detected obs')
                    
    #Stop robot
    Controller.robot_stop(vrep, clientID, robot_LeftMotorHandle, robot_RightMotorHandle)    
        
    if counter > params.cnn_count:                    
        cv2.namedWindow('depth', cv2.WINDOW_NORMAL)
        cv2.imshow('depth',buffer/255.0)
        cv2.waitKey(5000)
        print('Going back to', path[-1])
        l_goal = path[-1]
        while Planner.at_goal(state[0:2], l_goal) == False:
            [V,W] = Controller.gtg(state,l_goal)
            Controller.robot_setvel(V,W, vrep, clientID, robot_LeftMotorHandle, robot_RightMotorHandle)
            state = Controller.localise(vrep, clientID, robot_Handle)  
        Controller.robot_stop(vrep, clientID, robot_LeftMotorHandle, robot_RightMotorHandle) 
        cv2.destroyAllWindows() 
        print('Reached back')

        print('Updating map info')
        #If already existing then just update
        for node in params.obs_list:
            if [node.x, node.y] == l_goal:
                node.obs_actionlist = node.obs_actionlist + [n_action]
                node.obs_actionlist = list(set(node.obs_actionlist))
                flag_exist = True
                break            
        #If not existing then just append    
        if flag_exist == False:
            temp = params.obsinfo(l_goal,[n_action])
            params.obs_list.append(temp) 
    
if Planner.at_goal(state[0:2], main_goal):
    print('Reached destination') 
else:
    print('Unable to reach destination')
print('Stopping the simulation')    
#Stop everything and end
Controller.robot_stop(vrep, clientID, robot_LeftMotorHandle, robot_RightMotorHandle)    
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
time.sleep(2.0)

#close data recording file
params.filename.close()
