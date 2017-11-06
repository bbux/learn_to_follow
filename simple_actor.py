""" module for holding actor that behaves in simple manner """
import numpy as np


def noise():
    return np.random.ranf() - 0.3

class SimpleActor(object):
    """ class for simple action in response to robot state """

    def __init__(self, goal_distance, base_velocity=3.0):
        self.goal_distance = goal_distance
        self.base_velocity = base_velocity

    def choose_action(self, state):
        """ choose actions given the current state

            returns: tuple of (left motor velocity, right motor velocity)
        """

        # strategy is to keep moving forward with proportional velocity to the delta in distance
        dist_delta = state[0] - self.goal_distance
        vleft = -(dist_delta * self.base_velocity) + noise()
        vright = vleft + noise()
        actions = (vleft, vright)
        return actions
