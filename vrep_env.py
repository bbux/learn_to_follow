#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
module for storing and interacting with the vrep environment
"""

import sys
import math
import vrep


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
    for i in range(0, len(usensors)-1):
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
        """ turn state into more consumable form

            returns: list with [ dist, theta, vleft, vright,  s1 ,s2, ..., sN ]
        """
        return [self.dist, self.theta, self.vleft, self.vright] + list(self.sensor_readings)


class Env(object):
    """ Class for encapsulating a vrep environment """

    def __init__(self, vleft=0, vright=0):
        """ initialize the vrep evironment

            params: vleft - initial left motor velocity
                    vright - initial right motor velocity
        """
        self.client_id = setup_vrep()

        _, self.target_handle = get_handle(self.client_id, 'Sphere')
        _, self.ref_frame = get_handle(self.client_id, 'Pioneer_p3dx')
        _, self.motor_left = get_handle(self.client_id, 'Pioneer_p3dx_leftMotor')
        _, self.motor_right = get_handle(self.client_id, 'Pioneer_p3dx_rightMotor')
        self.usensors = get_sensor_handles(self.client_id, "Pioneer_p3dx_ultrasonicSensor", 16)
        self.vleft = vleft
        self.vright = vright


    def get_state(self):
        """ gets the current state of the environment

            returns: State - of current environment
        """
        return read_state(self.client_id, self.target_handle, self.ref_frame, self.vleft, self.vright, self.usensors)

    def set_velocity(self, vleft, vright):
        """ set the velocity of the left and right robot motors

            params: vleft - left motor velocity
                    vright - right motor velocity
        """
        self.vleft = vleft
        self.vright = vright
        vrep.simxSetJointTargetVelocity(self.client_id, self.motor_left, vleft, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(self.client_id, self.motor_right, vright, vrep.simx_opmode_oneshot_wait)

    def stop(self):
        """ stop the vrep environment """
        # Now close the connection to V-REP:
        vrep.simxFinish(self.client_id)


def make():
    """ makes a new vrep environment """
    return Env()
