from algos import ddpg_her
import torch as th
import gym
import gym_hsr_gazebo


def env_fn(): return gym.make('HsrbPush-v0')


ac_kwargs = dict(hidden_sizes=[512, 512, 512], activation=th.nn.ReLU)

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
    start_steps=0,
    update_after=0,
    gamma=0.95,
    q_lr=0.001,
    pi_lr=0.001,
    num_additional_goals=4,
    goal_selection_strategy='future',
    logger_kwargs=logger_kwargs,
    demos=['data/demos/1.hdf5', 'data/demos/2.hdf5', 'data/demos/3.hdf5', 'data/demos/4.hdf5',
           'data/demos/5.hdf5', 'data/demos/6.hdf5', 'data/demos/7.hdf5', 'data/demos/8.hdf5'])
