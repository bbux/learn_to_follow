# Learn to Follow - A Reinforcement Learning Approach

## Goals
The goal of this project is to build a neural network that can learn a control function that will allow a robot
to follow a target at a specified distance and angle while avoiding obstacles.  The code uses a DDPG (Deep Deterministic Policy Gradients)
network adapted from: https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow/blob/master/experiments/Robot_arm/DDPG.py

The original Environment is a Robot Arm with two joints trying to touch a specific location. This was adapted to work with a V-REP simulation 
as the environment and to follow a simulated target at a specified distance and angle. A reward function was created based on how well the
robot met these goals.

## Dependencies
 - tensorflow
 - V-REP

## Running

To run, first set up V-REP and load the scene from vrep_scenes/straightline_no_obs_follow.ttt.  This scene will have the core components
needed to do train the basic scenarios that have been created up to this point.  There is a customized model for the pioneer p3dx in the
vrep_models folder.  The model that comes with V-REP has a built in script that conflicts with the custom environment controller. There may
be places in the code where the location of the model is hard coded.  Be sure to update these. Next start the V-REP simulation then execute
either the training or testing ddpg modes (see below).

### Training
Use the --mode train flag to initiate training mode.  For example:

```bash
./ddpg.py --mode train --save-path ../vrep-train --load-mem-path ~/learn_to_follow_mem_10k_in_a_circle
```

The code above will use the prefilled memory buffer at the specified path to seed the learning process.  The results of training will be stored
in the ../vrep-train folder.

### Testing
Use the --mode load to run the resulting model against the original or a new environment

```bash
./ddpg.py --mode load --save-path ../vrep-train
```

### Seeding the buffer
In order to create a memory buffer with helpful state action examples, it may be necessary to run an external program to build these up.
An example of this can be found in test_follow.py.
