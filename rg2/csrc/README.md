# Desing Notes

You have three sets of abstraction that we mainly care about:

First is the environment parallelizer: this is here to make use of the best threading possible, required due to python GIL. This is implemented as `CVecEnv` at `VecEnvWrapper`.

`CVecEnv` is implemented as vectorization of `WalkerEnv`, which defines individual property of the environments. Consequently, these are independent of each other.

`WalkerEnv` should be able to simulate various locomotion environments.

## On Initialization

These should be able to accept various arguments during initialization including:

1. PD configurations, such as

   1. Limits, STD, AVG of the joint configs.
   2. which joints to activate (by default, all joints) thus the action mapping configurations
   3. Set of PD gains

2. Reward Configurations, such as:

   1. Reward toggles / Coefficients
   2. Contact Configurations (Sets of body to encourage contact with / avoid contact with)

3. Observation / World such as:

   1. Observation toggles / Coefficients
   2. Privileged Knowledge Configurations (Sets of ground truth to return)
   3. History Buffer configurations (how many set of previous infos to return)
   4. Heightmap Scaning configurations

4. Curriculum, Noise Configurations, such as:
   1. Curriculum configurations (how to change the environment during training)
   2. Noise configurations (how to add noise to the environment during training)

## Per Step

`WalkerEnv` should be able to handle various per-step arguements, such as:

1. Curriculum configurations, such as:

   1. How much noise to add to observation / environment / action
   2. How difficult to make the environment, such as time noise, lack of history buffer, etc

2. Reward configurations, such as:

   1. Reward coefficients, reward clipping
   2. Reward toggles, such as whether to use contact reward, etc

3. Terrain / World configurations
   1. Set of terrains / world heighmap information
   2. Physics configuration, such as gravity, timestep, robot mass, etc
   3. History Buffer configurations (how many set of previous infos to return)
