#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
module for holding the TargetMover class
"""
import sys
import vrep


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
        movex, movey = self.get_next_pos()

        # what is our current absolute position
        _, pos = vrep.simxGetObjectPosition(self.client_id, self.handle, -1, vrep.simx_opmode_oneshot_wait)
        x = pos[0]
        y = pos[1]
        z = pos[2]

        vrep.simxSetObjectPosition(self.client_id, self.handle, -1, (x+movex, y+movey, z), vrep.simx_opmode_oneshot_wait)
        self._index = self._index + 1

    def get_next_pos(self):
        """ what is the next x and y position """
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

        return (movex, movey)
