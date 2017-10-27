""" module for holding actor that uses linear regression to learn actions """

import tensorflow as tf

class LR_Actor(object):
    """ class for simple action in response to robot state """

    def __init__(self, env, goal_distance, base_velocity=5.0):
        self.goal_distance = goal_distance
        self.base_velocity = base_velocity
        self.sess = tf.Session()
        # weights
        self.W = tf.Variable(tf.random_uniform([1], -1.0, 1.0))
        # biases
        self.b = tf.Variable(tf.zeros([1]))
        # input
        self.x = tf.placeholder(shape=[1,env.state_space], dtype=tf.float32)
        # prediction distance
        y = self.W * self.x + self.b
        loss = tf.reduce_mean(tf.square(y - goal_distance))
        optimizer = tf.train.GradientDescentOptimizer(0.01)
        self.train = optimizer.minimize(loss)
        
        self.sess.run(tf.global_variables_initializer())

    def choose_action(self, state):
        """ choose actions given the current state
            returns: tuple of (left motor velocity, right motor velocity)
        """
        self.sess.run(self.train, feed_dict={"x": state, "y_data": state[0]})

        
