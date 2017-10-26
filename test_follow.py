#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
module for training and testing the follow models
"""
import time
import vrep_env
from target_mover import TargetMover
from simple_actor import SimpleActor


def main():
    """ run the tests """

    # how far we want to be from the target
    goal_distance=1

    # what velocity to scale by
    base_velocity = 5.0

    # create the vrep environment
    env = vrep_env.make(goal_distance)

    # path to follow, just keep moving right
    path = ["R"] * 20 + ["exit"]
 
    # moves the target we are trying to fallow
    mover = TargetMover(env.client_id, target_handle=env.target_handle, path=path)

    # tells us what actions to take
    actor = SimpleActor(goal_distance, base_velocity)

    # WORK starts here
    state = env.reset()

    while True:
        done = mover.step()
        if done:
            break

        actions = actor.choose_action(state)
        state, reward = env.step(actions)

        time.sleep(1)

    env.stop()


if __name__ == "__main__":
    main()
