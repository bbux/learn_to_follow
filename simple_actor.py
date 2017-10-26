""" module for holding actor that behaves in simple manner """
class SimpleActor(object):
    """ class for simple action in response to robot state """

    def __init__(self, goal_distance, base_velocity=5.0):
        self.goal_distance = goal_distance
        self.base_velocity = base_velocity

    def choose_action(self, state):
        """ choose actions given the current state

            returns: tuple of (left motor velocity, right motor velocity)
        """

        # strategy is to keep moving forward with proportional velocity to the delta in distance
        dist_delta = state[0] - self.goal_distance
        vleft = dist_delta * self.base_velocity
        vright = vleft
        print("Delta distance: %f, Velocity: %f" % (dist_delta, vleft))
        actions = (vleft, vright)
        return actions
