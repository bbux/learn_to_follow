""" module for holding actor that behaves in simple manner """
import math
import numpy as np


def noise():
    return 0.0
    #return np.random.ranf()/10.0 - 0.05

class SimpleActor(object):
    """ class for simple action in response to robot state """

    def __init__(self, goal_distance, base_velocity=0.5):
        self.goal_distance = goal_distance
        self.base_velocity = base_velocity

    def update(self, s, a, r, s_):
        """ update actor policy base on state and reward

            params: s  - original state
                    a  - action
                    r  - reward for action
                    s_ - next state
        """
        # noop
        return

    def choose_action(self, state):
        """ choose actions given the current state

            returns: tuple of (left motor velocity, right motor velocity)
        """
        xpos = state[0]
        ypos = state[1]
        dist = math.sqrt(xpos**2 + ypos**2)
        relative = math.degrees(state[3])
        #delta_angle = orient - relative
        delta_angle = -relative
        dist_delta = dist - self.goal_distance
    
        if (abs(delta_angle) < 1):
            scale = 1.25
        elif (abs(delta_angle) < 3):
            scale = 1.5
        elif (abs(delta_angle) < 9):
            scale = 1.75
        elif (abs(delta_angle) < 18):
            scale = 2.0
        elif (abs(delta_angle) < 36):
            scale = 2.25
        else:
            scale = 2.5
            
        """ chart:
            Dist | Angle | Turn | +/- Velocity
              <  |   <   |   L  |   -
              <  |   >   |   R  |   -
              >  |   <   |   R  |   +
              >  |   >   |   L  |   +
        """
        if (dist_delta < 0 and delta_angle < 0) or (dist_delta > 0 and delta_angle > 0):
            vleft = scale * self.base_velocity + noise()
            vright = self.base_velocity + noise()
        else:
            vright = scale * self.base_velocity + noise()
            vleft = self.base_velocity + noise()
        
        if (dist_delta < 0):
            vleft = -vleft
            vright = -vright
    
        return (vleft, vright)
