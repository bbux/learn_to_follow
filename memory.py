""" Module for holding memory class """
import numpy as np


class Memory(object):
    """ class for storing transition tuples """

    def __init__(self, capacity, dims):
        """ init memory

            params: capacity - size of memory
                    dims     - dimensions of memory to store
        """
        self.capacity = capacity
        self.data = np.zeros((capacity, dims))
        self.pointer = 0

    def store_transition(self, s, a, r, s_):
        """ store the transition specified by the current state, action, reward, and next state
            if memory is at capacity, replaces an older data point with the one provided

            params: s  - original state
                    a  - action
                    r  - reward for action
                    s_ - next state
        """
        transition = np.hstack((s, a, [r], s_))
        index = self.pointer % self.capacity  # replace the old memory with new memory
        self.data[index, :] = transition
        self.pointer += 1

    def sample(self, n):
        """ sample the memory to get n examples 
        
            params: n - number of samples
            
            return: n samples of the
        """
        
        assert self.pointer >= self.capacity, 'Memory has not been fulfilled'
        indices = np.random.choice(self.capacity, size=n)
        return self.data[indices, :]

    def save(self, location):
        """ save the memory to the provided location

            params: location - to store memory
        """
        with open(location, "wb") as file_handle:
            np.save(file_handle, self.data)

    def load(self, location):
        """ load the memory from the provided location

            params: location - to load memory from
        """
        with open(location, "rb") as file_handle:
            self.data = np.load(file_handle)
