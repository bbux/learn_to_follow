#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
module for training and testing the follow models
"""
import time
import vrep_env
from target_mover import TargetMover

##########################################################
#                   START SCRIPT
##########################################################
def main():
    """ run the tests """
    env = vrep_env.make()

    # path to follow, just keep moving right
    path = ["R"] * 20 + ["exit"]
    mover = TargetMover(env.client_id, target_handle=env.target_handle, path=path)

    goal_distance = 1
    base_velocity = 5.0
    vleft = 0
    vright = 0

    while True:
        mover.step()

        state = env.get_state()

        dist_delta = state.dist - goal_distance
        vleft = dist_delta * base_velocity
        vright = vleft
        print("Delta distance: %f, Velocity: %f" % (dist_delta, vleft))

        env.set_velocity(vleft, vright)

        time.sleep(1)


if __name__ == "__main__":
    main()
