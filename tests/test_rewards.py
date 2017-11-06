import rewards
import math

class FakeState(object):
    def __init__(self, dist, theta):
        self.dist = dist
        self.theta = theta

def test_orientation_aligned_little_change():
    rewarder = rewards.OrientationAlignedRewarder(1, 0, 1, -1)
    orig_state = FakeState(1, math.radians(1))
    new_state = FakeState(1, math.radians(-1))
    reward = rewarder.calculate_reward(orig_state, new_state)
    # moved from +1 degree off to -1 degree off, not significant enough
    assert reward == 0

def test_orientation_aligned_pos_reward():
    rewarder = rewards.OrientationAlignedRewarder(1, 0, 1, -1)
    orig_state = FakeState(1, math.radians(-15))
    new_state = FakeState(1, math.radians(5))
    reward = rewarder.calculate_reward(orig_state, new_state)
    # moved from pretty for off to better angle, should get reward
    assert reward == 1

def test_orientation_aligned_neg_reward():
    rewarder = rewards.OrientationAlignedRewarder(1, 0, 1, -1)
    orig_state = FakeState(1, math.radians(15))
    new_state = FakeState(1, math.radians(25))
    reward = rewarder.calculate_reward(orig_state, new_state)
    # moved from pretty for to even farther, we do not want to encourage this
    assert reward == -1

def test_orientation_aligned_large_angles():
    rewarder = rewards.OrientationAlignedRewarder(1, 0, 1, -1)
    orig_state = FakeState(1, math.radians(185))
    new_state = FakeState(1, math.radians(190))
    reward = rewarder.calculate_reward(orig_state, new_state)
    # moved very far to a little closer
    assert reward == 1

def test_graduated_reward_distance():
    rewarder = rewards.GraduatedReward(1, 0, 1, -1)
    orig_state = FakeState(1, 0)
    
    for delta, expected_reward in rewards.DISTANCE_REWARD_MATRIX:
        # slightly ahead of cutoff
        dist = 1 - delta + .01
        new_state = FakeState(dist, math.radians(180))
        reward = rewarder.calculate_reward(orig_state, new_state)
        assert reward ==  expected_reward
        # slightly behind cutoff on other side
        dist = 1 + delta - .01
        new_state = FakeState(dist, math.radians(180))
        reward = rewarder.calculate_reward(orig_state, new_state)
        assert reward ==  expected_reward

def test_graduated_reward_angle():
    rewarder = rewards.GraduatedReward(1, 0, 1, -1)
    orig_state = FakeState(1, 0)
    
    for delta, expected_reward in rewards.ANGLE_REWARD_MATRIX:
        # slightly below angle cutoff on positive side
        angle = 0 + delta - .01
        new_state = FakeState(10, math.radians(angle))
        reward = rewarder.calculate_reward(orig_state, new_state)
        assert reward ==  expected_reward
        # slightly above angle cutoff on negative side
        angle = 0 - delta + .01
        new_state = FakeState(10, math.radians(angle))
        reward = rewarder.calculate_reward(orig_state, new_state)
        assert reward ==  expected_reward

def test_graduated_reward_both_bad():
    rewarder = rewards.GraduatedReward(1, 0, 1, -1)
    orig_state = FakeState(1, 0)
    new_state = FakeState(10, math.radians(180))
    reward = rewarder.calculate_reward(orig_state, new_state)
    assert reward ==  -1
