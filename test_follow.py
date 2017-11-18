#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
module for training and testing the follow models
"""
import vrep_env
from target_mover import TargetMover
from simple_actor import SimpleActor
import memory
import rewards


MEMORY_CAPACITY=10000
MAX_EPISODES=5000

def main():
    # how far we want to be from the target
    goal_distance=1

    # create the vrep environment
    env = vrep_env.make(goal_distance, rewarder=rewards.graduated(goal_distance))
    
    mem = memory.Memory(MEMORY_CAPACITY, dims=2 * env.state_dim + env.action_dim + 1)

    # path to follow, just keep moving right
    #path = ["R"] * 20 + ["exit"]
    path = ["F"] * 7 + ["R"] * 7 + ["B"] * 7 + ["L"] * 7 + ["exit"]
 
    # moves the target we are trying to fallow
    mover = TargetMover(env.client_id, target_handle=env.target_handle, path=path)

    # tells us what actions to take
    actor = SimpleActor(goal_distance)
    
    cumulative_reward = 0

    for ep in range(MAX_EPISODES):
        print("Episode: %d, Memory: %d, Average Reward: %f"  % (ep, mem.pointer, cumulative_reward/(ep+1)))
        # done loading sample points
        if mem.pointer > MEMORY_CAPACITY:
            break;
        # reset environment and mover
        mover.reset()
        s = env.reset()
    
        # keep going tell mover or env say stop
        while True:
            done = mover.step()
            if done:
                break
    
            a = actor.choose_action(s)
            s_, r, done = env.step(a)
            mem.store_transition(s, a, r, s_)
            s = s_
            cumulative_reward += r
            
            print("X Pos: %f, Y Pos : %f, Velocity: L %f R %f, Reward: %f" % 
                (s[0], s[1], a[0], a[1], r))
    
            if done:
                break;
    
            #time.sleep(0.1)

    print("Average Reward: " + str(cumulative_reward/MAX_EPISODES))
    mem.save("/tmp/learn_to_follow_mem_10k")
    env.stop()


if __name__ == "__main__":
    main()
