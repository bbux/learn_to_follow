#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
module for storing and interacting with the vrep environment
"""

import sys
import math
import vrep
import numpy as np
import time
import rewards

def setup_vrep():
    """ sets up and connects to the vrep server
        returns: client_id - the client_id for interacting with vrep
    """
    vrep.simxFinish(-1) # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5) # Connect to V-REP

    if client_id != -1:
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
    usensors = [-1] * num_sensors
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
    for i in range(0, len(usensors) - 1):
        (_, _, detected_point, _, _) = vrep.simxReadProximitySensor(client_id, usensors[i],
                                                                    vrep.simx_opmode_oneshot_wait)
        dist = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
        readings[i] = dist
    return readings


def read_state(client_id, target_handle, ref_frame, vleft, vright, usensors):
    """ reads the distance measured by each sensor

        params: client_id - to connect to vrep server with
                target_handle - handle of object being followed
                ref_frame - reference frame for distance and angle calculations, usually the robot
                vleft - current left motor velocity
                vright - current right motor velocity
                usensors - list of sensor handles

        returns: state - current state of the robot
    """
    _, pos = vrep.simxGetObjectPosition(client_id, target_handle, ref_frame, vrep.simx_opmode_oneshot_wait)
    _, orient = vrep.simxGetObjectOrientation (client_id, target_handle, ref_frame, vrep.simx_opmode_oneshot_wait)
    
    dist = calculate_distance(pos)
    return State(dist, orient[2], vleft, vright, read_sensors(client_id, usensors))


def calculate_distance(pos):
    """ calculates the distance from the current measured position

        params: pos - list of (x, y, z) measurements

        returns: distance - euclidean distance to this point
    """
    return math.sqrt(pos[0]**2 + pos[1]**2)


class State(object):
    """ for storing the state elements of the robot """
    def __init__(self, dist, theta, vleft, vright, sensor_readings=None):
        """ constructs the state object

            params: dist - the distance from the target
                    theta - the relative orientation to the target in radians
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

    def to_array(self):
        """ turn state into more consumable form

            returns: list with [ dist, theta, vleft, vright,  s1 ,s2, ..., sN ]
        """
        #return np.asarray([self.dist, self.theta, self.vleft, self.vright] + list(self.sensor_readings))
        #return np.asarray([self.dist, self.theta, self.vleft, self.vright])
        return np.asarray([self.dist, self.theta])

def get_reset(client_id, handle):
    """ gets a Reset object holding the original position and orientation of the object """
    _, pos = vrep.simxGetObjectPosition(client_id, handle, -1, vrep.simx_opmode_oneshot_wait)
    _, orient = vrep.simxGetObjectOrientation(client_id, handle, -1, vrep.simx_opmode_oneshot_wait)
    return Reset(handle, pos, orient)
    
def reset_object(client_id, reset):
    """ puts object back to original position """
    vrep.simxSetObjectPosition(client_id, reset.handle, -1, reset.pos, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectOrientation(client_id, reset.handle, -1, reset.orient, vrep.simx_opmode_oneshot_wait)
        
    
class Reset(object):
    """ for storing reset information """
    def __init__(self, handle, pos, orient):
        self.handle = handle
        self.pos = pos
        self.orient = orient
        

class VREP_Env(object):
    """ Class for encapsulating a vrep environment """
    # distance, theta, vleft, vright, +16 distance sensors
    state_dim = 2
    # left motor velocity and right motor velocity
    action_dim = 2
    # max min velocity change?
    action_bound = [-1, 1]

    def __init__(self, rewarder, vleft=0, vright=0, goal_distance=1, max_delta=1, sleep_time=0.1):
        """ initialize the vrep evironment

            params: vleft - initial left motor velocity
                    vright - initial right motor velocity
        """
        self.client_id = setup_vrep()

        _, self.target_handle = get_handle(self.client_id, 'Sphere')
        self._load_robot_handles()
        self.rewarder = rewarder
        self.vleft = vleft
        self.vright = vright
        self.goal_distance = goal_distance
        self.max_delta = max_delta
        self.sleep_time = sleep_time
        self.target_reset = get_reset(self.client_id, self.target_handle)

    def get_state(self):
        """ gets the current state of the environment

            returns: State - of current environment
        """
        return read_state(self.client_id, self.target_handle, self.ref_frame, self.vleft, self.vright, self.usensors)

    def step(self, actions):
        """ take an action

            params: action[0] = vleft - left motor velocity
                    action[1] = vright - right motor velocity

            returns: (state, reward)
        """
        orig_state = self.get_state()
        self.vleft = actions[0]
        self.vright = actions[1]
        vrep.simxSetJointTargetVelocity(self.client_id, self.motor_left, self.vleft, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(self.client_id, self.motor_right, self.vright, vrep.simx_opmode_oneshot_wait)
        time.sleep(self.sleep_time)
        new_state = self.get_state()
        # never done
        return (new_state.to_array(), self.rewarder.calculate_reward(orig_state, new_state), self._is_done(new_state))
        
    def reset(self):
        """ reset the state 

            retruns: the current state after reset
        """
        reset_object(self.client_id, self.target_reset)
        vrep.simxRemoveModel(self.client_id, self.ref_frame, vrep.simx_opmode_oneshot_wait)
        vrep.simxLoadModel(self.client_id, "/home/user/V-REP/models/robots/mobile/pioneer_p3dx_script_disabled.ttm",
                           1, vrep.simx_opmode_oneshot_wait)
        self._load_robot_handles()
        self.vleft = 0
        self.vright = 0
        
        return self.get_state().to_array()
        
    def stop(self):
        """ stop the vrep environment """
        # Now close the connection to V-REP:
        vrep.simxFinish(self.client_id)

    def _is_done(self, state):
        """ have we deviated outside of acceptable range """
        delta = abs(state.dist - self.goal_distance)
        return delta > self.max_delta

    def _calculate_reward(self, orig_state, new_state):
        """ calculates the reward based on if the action is moving the robot closer to the goal or farther away """
        delta_orig = abs(orig_state.dist - self.goal_distance)
        delta_new = abs(new_state.dist - self.goal_distance)
        reward = 0
        # this means we are going in the correct direction
        if delta_new < delta_orig:
            reward += 1
            # if one length away add extra bonus
            if delta_new < self.goal_distance:
                reward += 1
            # if half a length add another bonus
            if delta_new < self.goal_distance/2.0:
                reward += 1
        else:
            reward += -1
       
        return reward

    def _load_robot_handles(self):
        """ loads/reloads the handles for the robot """
        _, self.ref_frame = get_handle(self.client_id, 'Pioneer_p3dx')
        _, self.motor_left = get_handle(self.client_id, 'Pioneer_p3dx_leftMotor')
        _, self.motor_right = get_handle(self.client_id, 'Pioneer_p3dx_rightMotor')
        self.usensors = get_sensor_handles(self.client_id, "Pioneer_p3dx_ultrasonicSensor", 16)


def make(goal_distance, rewarder=None):
    """ makes a new vrep environment 
        
        params: goal_distance - the desired distance to the target
    """
    if rewarder is None:
        rewarder = rewards.default(goal_distance)
    return VREP_Env(rewarder, goal_distance=goal_distance)
