#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 14:49:07 2017

@author: user
"""

import vrep
import sys
import time
import math


def setup_vrep():
    """ sets up and connects to the vrep server
        returns: client_id - the client_id for interacting with vrep
    """
    vrep.simxFinish(-1) # just in case, close all opened connections
    client_id=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
    
    if client_id!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect')
    return client_id

   
def get_handle(client_id, handle_name):
    """ get the vrep handle for the provided handle name

        params: client_id - to connect to vrep server with
                handle_name - name of handle to lookup

        returns: rc, handle - retrun conde and object handle
    """
    return vrep.simxGetObjectHandle(client_id, 
                                    handle_name,
                                    vrep.simx_opmode_oneshot_wait)


def get_sensor_handles(client_id, base_name, num_sensors):
    """ get list of sensor handles with names that start with base 

        params: client_id - to connect to vrep server with
                base_name - base of object sensor name
                num_sensors - number of sensors that start with this base name

        returns: usensors - list of sensor handles for sensors 1 to N 
   """
    usensors=[-1] * num_sensors
    for i in range(1, num_sensors):
        _, usensors[i] = vrep.simxGetObjectHandle(client_id, base_name + str(i), 
                                                  vrep.simx_opmode_oneshot_wait)
    return usensors


def read_sensors(client_id, usensors):
    """ reads the distance measured by each sensor

        params: client_id - to connect to vrep server with
                usensors - list of sensor handles

        returns: readings - list of sensor distance readings
    """
    readings = [0] * len(usensors)
    for i in range(0, len(usensors)-1):
        (_, _, detected_point, _, _) = vrep.simxReadProximitySensor(client_id, usensors[i], 
                                                                    vrep.simx_opmode_oneshot_wait)
        dist = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
        readings[i] = dist
    return readings
 
       
def get_state(client_id, target_handle, ref_frame, vleft, vright, usensors):
    """ reads the distance measured by each sensor

        params: client_id - to connect to vrep server with
                target_handle - handle of object being followed
                ref_frame - reference frame for distance and angle calculations, usually the robot
                vleft - current left motor velocity
                vright - current right motor velocity
                usensors - list of sensor handles

        returns: state - current state of the robot
    """
    rc, pos = vrep.simxGetObjectPosition(client_id, target_handle, ref_frame, vrep.simx_opmode_oneshot_wait)
    dist = calculate_distance(pos)
    angle = calculate_angle(pos)
    return State(dist, angle, vleft, vright, read_sensors(client_id, usensors))


def calculate_distance(pos):
    """ calculates the distance from the current measured position

        params: pos - list of (x, y, z) measurements

        returns: distance - euclidean distance to this point
    """
    return math.sqrt(pos[0]**2 + pos[1]**2)


def calculate_angle(pos):
    """ calculates the angle from the current measured position

        params: pos - list of (x, y, z) measurements

        returns: angle - in degrees
    """
    return math.degrees(math.atan2(pos[1], pos[0]))


class TargetMover(object):
    """ Class for moving the target along a desired path """
    def __init__(self, client_id, target_handle, path, increment=0.1):
        """ Initialize the target mover
    
            params: client_id - to connect to vrep server with
                    target_handle - handle of object being followed
                    path - list of [RLFB] instruction for what direction to move
                    increment - how far to move for each step
    
            returns: state - current state of the robot
        """
        self.client_id = client_id
        self.handle = target_handle
        self.path = path
        self.increment = increment
        self._index = 0
        

    def step(self):
        """ move the target to the next positions specified in the path """
        val = self.path[self._index]
        if val == "exit":
            sys.exit("Sim Stopped")
        elif val == "L":
            movex = -self.increment
            movey = 0.0
        elif val == "R":
            movex = self.increment
            movey = 0.0
        elif val == "F":
            movey = self.increment
            movex = 0.0
        elif val == "B":
            movey = -self.increment
            movex = 0.0
        
        # what is our current absolute positiono        
        rc, pos = vrep.simxGetObjectPosition(self.client_id, self.handle, -1, vrep.simx_opmode_oneshot_wait)
        x = pos[0]
        y = pos[1]
        z = pos[2]
    
        vrep.simxSetObjectPosition(self.client_id, self.handle, -1, (x+movex, y+movey, z), vrep.simx_opmode_oneshot_wait)
        self._index = self._index + 1


class State(object):
    """ for storing the state elements of the robot """
    def __init__(self, dist, theta, vleft, vright, sensor_readings=None):
        """ constructs the state object
    
            params: dist - the distance from the target
                    theta - the angle to the target in degrees
                    vleft - current left motor velocity
                    vright - current right motor velocity
                    sensor_readings - list of sensor distance readings
    
            returns: state - current state of the robot
        """
        self.dist = dist
        self.theta = theta
        self.vleft = vleft
        self.vright = vright
        self.sensor_readings = sensor_readings
        
    def to_list(self):
        return [self.dist, self.theta] + list(self.sensor_readings)


##########################################################
#                   START SCRIPT
##########################################################
client_id = setup_vrep()

_, sphere_handle = get_handle(client_id,'Sphere')
_, ref_frame = get_handle(client_id, 'Pioneer_p3dx')
_, motor_left = get_handle(client_id,'Pioneer_p3dx_leftMotor')
_, motor_right = get_handle(client_id, 'Pioneer_p3dx_rightMotor')
usensors = get_sensor_handles(client_id, "Pioneer_p3dx_ultrasonicSensor", 16)

# path to follow, just keep moving right
path = [ "R" ] * 20 + [ "exit" ]
mover = TargetMover(client_id, target_handle=sphere_handle, path=path)


goal_distance = 1
base_velocity=5.0
vleft=0
vright=0

while True:
    mover.step()
    
    state = get_state(client_id, sphere_handle, ref_frame, vleft, vright, usensors)
    
    dist_delta = state.dist - goal_distance
    vleft = dist_delta * base_velocity
    vright = vleft
    print("Delta distance: %f, Velocity: %f" % (dist_delta, vleft))
     
    vrep.simxSetJointTargetVelocity(client_id, motor_left, vleft, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(client_id, motor_right, vright, vrep.simx_opmode_oneshot_wait)

    time.sleep(1)


# Now close the connection to V-REP:
vrep.simxFinish(client_id)