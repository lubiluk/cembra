from algos import ddpg_her
import torch as th
import gym
import gym_hsr_gazebo


def env_fn(): return gym.make('HsrbPush-v0')


ac_kwargs = dict(hidden_sizes=[64, 64], activation=th.nn.ReLU)

logger_kwargs = dict(
    output_dir='data/hsrb_push_ddpg_her_0',
    exp_name='hsrb_push_ddpg_her_0')

ddpg_her(
    env_fn=env_fn,
    ac_kwargs=ac_kwargs,
    steps_per_epoch=15000,
    epochs=200,
    batch_size=256,
    replay_size=1000000,
    start_steps=1000,
    gamma=0.95,
    q_lr=0.001,
    pi_lr=0.001,
    num_additional_goals=4,
    goal_selection_strategy='future',
    logger_kwargs=logger_kwargs,
    demos=['data/1.hdf5', 'data/2.hdf5', 'data/3.hdf5', 'data/4.hdf5',
           'data/5.hdf5', 'data/6.hdf5', 'data/7.hdf5', 'data/8.hdf5'])
