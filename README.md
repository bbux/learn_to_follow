# Learn to Follow - A Reinforcement Learning Approach

The goal of this project is to build a neural network that can learn a control function that will allow a robot
to follow a target at a specified distance while avoiding obstacles.  The code uses a DDPG (Deep Determanistic Policy Gradients)
network adapted from: https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow/blob/master/experiments/Robot_arm/DDPG.py

The original Environment is a Robot Arm with two joints trying to touch a specific location. Adapted to work with vrep simulation 
as environment to follow a target at a specified distance and angle.

## Dependencies
 - tensorflow
 - V-REP

## Running

To run first set up V-REP and load the scene from vrep_scenes/straightline_no_obs_follow.ttt.  This scene will have to core components
needed to do the simulations that have been created up to this point.  There is a customized model for the pioneer p3dx in the
vrep_models folder.  The model that comes with V-REP has a built in script that conflicts with the custom environment controller. Ther may
be places in the code where the locatioin of the model is hard coded.  Be sure to update these.

### Training
Use the --mode train flag to initiate training mode.  For example:

```bash
./ddpg.py --mode train --save-path ../vrep-train --load-mem-path ~/learn_to_follow_mem_10k_in_a_circle
```

### Testing
Use the --mode load to run the resulting model against the original or a new environment

```bash
./ddpg.py --mode load --save-path ../vrep-train
```

### Seeding the buffer
In order to create a memory buffer with helpful state action examples, it may be necessary to run an external program to build up these
memory examples.  An example of this can be found in test_follow.py.
