""" module for different reward calculation schemes """

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
    def __init__(self, goal_distance, pos_reward=1, neg_reward=-1, rewarders=None):
        self.goal_distance = goal_distance
        self.pos_reward = pos_reward
        self.neg_reward = neg_reward


class IncrementalReward(BaseRewarder):
    """ class that gives extra rewards based on how close to the goal a target is """

    def calculate_reward(self, orig_state, new_state):
        """ calculates the reward based on if the action  """
        delta_orig = abs(orig_state.dist - self.goal_distance)
        delta_new = abs(new_state.dist - self.goal_distance)
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


class MoveCloserRewarder(BaseRewarder):
    """ class that rewards the robot for reducing the delta in goal disance """

    def calculate_reward(self, orig_state, new_state):
        """ calculates the reward based on if the action is moving the robot closer to the goal or farther away """
        delta_orig = abs(orig_state.dist - self.goal_distance)
        delta_new = abs(new_state.dist - self.goal_distance)
        reward = 0
        # not a bit enough change
        if abs(delta_orig - delta_new) < 0.01:
            return 0
        # this means we are going in the correct direction
        if delta_new < delta_orig:
            reward += self.pos_reward
        else:
            reward += self.neg_reward

        return reward

def default(goal_distance, pos_reward=1, neg_reward=-1):
    """ return reward_calculator using the default rewarding scheme """
    rewarder1 = MoveCloserRewarder(goal_distance, pos_reward, neg_reward)
    rewarder2 = IncrementalReward(goal_distance, pos_reward, neg_reward)
    reward_calculator = RewardCalculator(rewarders=[rewarder1, rewarder2])
    return reward_calculator

class FakeState(object):
    def __init__(self, dist):
        self.dist = dist

def main():
    calculator = default(1)
    orig_state = FakeState(3.0)
    new_state = FakeState(3.3)
    
    reward = calculator.calculate_reward(orig_state, new_state)
    print("Calculated reward: " + str(reward))

if __name__ == "__main__":
    main()
