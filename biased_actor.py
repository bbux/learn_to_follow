""" module for holding actor that behaves in biased manner """
import numpy as np
import memory
import vrep_env

MEMORY_CAPACITY=10000


def noise():
    return np.random.ranf() - 0.3

class BiasedActor(object):
    """ class for biased action in response to robot state """

    def __init__(self, env, goal_distance, base_velocity=3.0):
        self.goal_distance = goal_distance
        self.base_velocity = base_velocity
        self.pos_mem = memory.Memory(2*MEMORY_CAPACITY/3, dims=2 * env.state_dim + env.action_dim + 1)
        self.neg_mem = memory.Memory(1*MEMORY_CAPACITY/3, dims=2 * env.state_dim + env.action_dim + 1)
        self.count = 0


    def choose_action(self, state):
        """ choose actions given the current state

            returns: tuple of (left motor velocity, right motor velocity)
        """
        self.count += 1
        # wait till there are enough samples to choose from, throw in a random new one every once in a while
        if self.pos_mem.pointer < 2000 or self.neg_mem.pointer == 1000 or self.count % 100 == 0:
            return self._random_action(state)
            
        if self.counter % 3 == 0:
            samp = self.neg_mem.sample(1)
        else:
            samp = self.pos_mem.sample(1)
        return samp[0][2]
            
    
    def update(self, s, a, r, s_):
        """ update actor policy based on state and reward

            params: s  - original state
                    a  - action
                    r  - reward for action
                    s_ - next state
        """
        if r > 0:
            self.pos_mem.store_transition(s, a, r, s_)
        else:
            self.neg_mem.store_transition(s, a, r, s_)

    def _random_action(self, state):
        # strategy is to keep moving forward with proportional velocity to the delta in distance
        dist_delta = state[0] - self.goal_distance
        vleft = -(dist_delta * self.base_velocity) + noise()
        vright = vleft + noise()
        actions = (vleft, vright)
        return actions
