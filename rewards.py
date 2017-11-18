""" module for different reward calculation schemes """
import math


DISTANCE_REWARD_MATRIX = [
    (0.1, 1.0),
    (0.2, 0.5),
    (0.3, 0.25),
    (0.4, 0.125),
    (0.5, 0.0625)
]

ANGLE_REWARD_MATRIX = [
    (3.0, 1.0),
    (6.0, 0.5),
    (9.0, 0.25),
    (12.0, 0.125),
    (15.0, 0.0625)
]

class RewardCalculator(object):
    """ class that calculates the reward using list of rewarders """
    def __init__(self, rewarders=None):
        if rewarders is None:
            self.rewarders = []
        else:
            self.rewarders = rewarders

    def calculate_reward(self, orig_state, new_state):
        """ giving the original and new state, calulate the reward using the configured rewarders """
        reward = 0
        reward += sum([r.calculate_reward(orig_state, new_state) for r in self.rewarders])
        return reward


class BaseRewarder(object):
    """ base class for storing field data """
    def __init__(self, goal_distance, goal_theta=0, pos_reward=1, neg_reward=-1):
        self.goal_distance = goal_distance
        self.goal_theta = goal_theta
        self.pos_reward = pos_reward
        self.neg_reward = neg_reward


class IncrementalReward(BaseRewarder):
    """ class that gives extra rewards based on how close to the goal a target is """

    def calculate_reward(self, orig_state, new_state):
        """ calculates the reward based on if the how far from goal the robot is  """
        delta_orig = abs(orig_state.dist() - self.goal_distance)
        delta_new = abs(new_state.dist() - self.goal_distance)
        reward = 0
        # this means we are going in the correct direction
        if delta_new < delta_orig:
            # if one length away add extra bonus
            if delta_new < self.goal_distance:
                reward += self.pos_reward
            # if half a length add another bonus
            if delta_new < self.goal_distance/2.0:
                reward += self.pos_reward

        return reward


class GraduatedReward(BaseRewarder):
    """ class that gives scaled rewards based on cloeness to goals """

    def calculate_reward(self, orig_state, new_state):
        """ calculates the reward based on graduated rewards  """
        # only care about new state, don't care how we got here
        dist_reward = 0
        actual_dist_delta = abs(new_state.dist() - self.goal_distance)
        for delta, graduated_reward in DISTANCE_REWARD_MATRIX:
            if actual_dist_delta < delta:
                dist_reward = graduated_reward
                break;
        
        angle_reward = 0
        actual_angle_delta = abs(math.degrees(new_state.theta - self.goal_theta))
        for delta, graduated_reward in ANGLE_REWARD_MATRIX:
            if actual_angle_delta < delta:
                angle_reward = graduated_reward
                break;
        # only good states are ones where distance and angle are good
        if not (dist_reward > 0 and angle_reward > 0):
            return -1
            
        return dist_reward + angle_reward
        

class MoveCloserRewarder(BaseRewarder):
    """ class that rewards the robot for reducing the delta in goal disance """

    def calculate_reward(self, orig_state, new_state):
        """ calculates the reward based on if the action is moving the robot closer to the goal or farther away """
        delta_orig = abs(orig_state.dist() - self.goal_distance)
        delta_new = abs(new_state.dist() - self.goal_distance)
        reward = 0
        # not a big enough change
        if abs(delta_orig - delta_new) < 0.01:
            return 0
        # this means we are going in the correct direction
        if delta_new < delta_orig:
            reward += self.pos_reward
        else:
            reward += self.neg_reward

        return reward


class OrientationAlignedRewarder(BaseRewarder):
    """ class that rewards the robot for reducing the delta in goal theta """

    def calculate_reward(self, orig_state, new_state):
        """ calculates the reward based on if the action is moving the robot closer to the goal or farther away """
        delta_orig = abs(math.degrees(orig_state.theta - self.goal_theta))
        if delta_orig > 180:
            delta_orig = delta_orig - 360
        delta_new = abs(math.degrees(new_state.theta - self.goal_theta))
        if delta_new > 180:
            delta_new = delta_new - 360

        delta_delta = abs(delta_orig - delta_new)
        if delta_delta < 1:
            return 0
 
        if abs(delta_new) <= abs(delta_orig):
            return self.pos_reward
        else:
            return self.neg_reward


def default(goal_distance, goal_theta=0, pos_reward=1, neg_reward=-1):
    """ return reward_calculator using the default rewarding scheme """
    rewarder1 = MoveCloserRewarder(goal_distance, goal_theta, pos_reward, neg_reward)
    rewarder2 = IncrementalReward(goal_distance, goal_theta, pos_reward, neg_reward)
    rewarder3 = OrientationAlignedRewarder(goal_distance, goal_theta, pos_reward, neg_reward)
    reward_calculator = RewardCalculator(rewarders=[rewarder1, rewarder2, rewarder3])
    return reward_calculator

def graduated(goal_distance, goal_theta=0, pos_reward=1, neg_reward=-1):
    """ return reward_calculator using the graduated rewarding scheme """
    rewarder = GraduatedReward(goal_distance, goal_theta, pos_reward, neg_reward)
    reward_calculator = RewardCalculator(rewarders=[rewarder])
    return reward_calculator


class FakeState(object):
    def __init__(self, dist, theta):
        self.dist = dist
        self.theta = theta

def main():
    calculator = default(1)
    orig_state = FakeState(3.0, math.radians(185))
    new_state = FakeState(3.3, math.radians(190))
    
    reward = calculator.calculate_reward(orig_state, new_state)
    print("Calculated reward: " + str(reward))

if __name__ == "__main__":
    main()
