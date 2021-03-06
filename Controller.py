# -*- coding: utf-8 -*-
"""
The controllers
"""
#Import libraries
import numpy as np

#Import files
import params

################Helper functions############################################### 
def rcrdr(data):
    #Function to record the pose data as it comes in
    params.filename.write(str(data))
    params.filename.write("\n")
    print(data)
    return

def localise(vrep, clientID, robot_Handle):
    #Function that will return the current location of the robot
    #PS. THE ORIENTATION IS IN RADIANS        
    
    res , robot_Position = vrep.simxGetObjectPosition(clientID, robot_Handle, -1 , vrep.simx_opmode_buffer)
    res , robot_Orientation = vrep.simxGetObjectOrientation(clientID, robot_Handle, -1 , vrep.simx_opmode_buffer)
     
    x = robot_Position[0]*1000.0 #in mm
    y = robot_Position[1]*1000.0 #in mm
    theta  = robot_Orientation[2]
    time_stamp = vrep.simxGetLastCmdTime(clientID)/1000.0
    if time_stamp%0.250 <= 0.01 and time_stamp > params.prev_time_stamp:
        rcrdr([robot_Position[0], robot_Position[1], robot_Position[2], time_stamp])
        params.prev_time_stamp = time_stamp
    return [x, y, theta]  
    
def robot_setvel(V,W, vrep, clientID, robot_LeftMotorHandle, robot_RightMotorHandle):
    #Function to set the linear and rotational velocity of robot           

    # 1. Limit v,w from controller to +/- of their max
    w = max(min(W, 0.3), -0.3)
    v = max(min(V, 0.7), -0.7)
            
    # 2. Compute desired vel_r, vel_l needed to ensure w
    Vr = ((2.0*v) + (w*params.robot_width))/(params.wheel_dia)
    Vl = ((2.0*v) - (w*params.robot_width))/(params.wheel_dia)
                        
    vel_r = Vr
    vel_l = Vl            
    
    # 3. Set velocity
    for i in range(4):
        vrep.simxSetJointTargetVelocity(clientID, robot_LeftMotorHandle[i], vel_l, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, robot_RightMotorHandle[i], vel_r, vrep.simx_opmode_streaming)
    
    return  
  
def robot_stop(vrep, clientID, robot_LeftMotorHandle, robot_RightMotorHandle):
    #Function to stop robot          
    
    for i in range(4):
        vrep.simxSetJointTargetVelocity(clientID, robot_LeftMotorHandle[i], 0.0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, robot_RightMotorHandle[i], 0.0, vrep.simx_opmode_streaming)
    
    #Reset error counters
    params.total_heading_error = 0.0
    params.prev_heading_error = 0.0
    
    return                                                                        
###################CONTROLLER###################################################
def gtg(state, goal):
    #The Go to goal controller
    
    #Unwrap
    x = state[0]
    y = state[1]
    theta = state[2]
    localgoal_x = goal[0]
    localgoal_y = goal[1]
     
    #determine how far to rotate to face the goal point
    #PS. ALL ANGLES ARE IN RADIANS
    dt = theta - (np.arctan2((localgoal_y - y), (localgoal_x - x)))
    #restrict angle to (-pi,pi)
    dt = ((dt + np.pi)%(2.0*np.pi)) - np.pi
    dt = ((dt*180.0)/np.pi)
        
    #control input for angular velocity
    W = (params.Kp*dt) + (params.Ki*params.total_heading_error) + (params.Kd*(dt - params.prev_heading_error))
    params.total_heading_error = params.total_heading_error + dt
    params.prev_heading_error = dt
  
    #find distance to goal
    #d = np.sqrt(((localgoal_x - x)**2) + ((localgoal_y - y)**2))
    
    #velocity parameters
    #velMult = 0.1#mm/s
    vmax = 0.1#mm
    
    #control input for linear velocity
    #V = ((np.arctan((d - distThresh))) - (np.arctan(dt)))*velMult
    V = -1.0*vmax*(1.0 - (2.0*(np.arctan(abs(dt))/np.pi)))
                                       
    return [V,W]                     
                 
def orient_robot(r_theta, g_theta):
    #The turn controller
     
    #determine how far to rotate to face the goal point
    #PS. ALL ANGLES ARE IN RADIANS
    dt = r_theta - g_theta 
    #restrict angle to (-pi,pi)
    dt = ((dt + np.pi)%(2.0*np.pi)) - np.pi
    dt = ((dt*180.0)/np.pi)
        
    #control input for angular velocity
    W = (params.Kp*dt) + (params.Ki*params.total_heading_error) + (params.Kd*(dt - params.prev_heading_error))
    params.total_heading_error = params.total_heading_error + dt
    params.prev_heading_error = dt
    
    #control input for linear velocity
    V = 0.0                                  
    return [V,W]                     
