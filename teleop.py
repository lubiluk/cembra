import gym
import gym_hsr_gazebo
import sys
import pygame
from algos.ddpg_her import ReplayBuffer
import time
import h5py
from algos import core

KEY_PAIRS = (
    (pygame.K_q, pygame.K_a),
    (pygame.K_w, pygame.K_s),
    (pygame.K_e, pygame.K_d),
    (pygame.K_r, pygame.K_f),
    (pygame.K_t, pygame.K_g),
    (pygame.K_y, pygame.K_h),
    (pygame.K_u, pygame.K_j),
    (pygame.K_i, pygame.K_k),
)

env = gym.make('HsrbPush-v0')

obs_dim = env.observation_space.spaces["observation"].shape
act_dim = env.action_space.shape[0]
goal_dim = env.observation_space.spaces["desired_goal"].shape

pygame.init()
size = width, height = 320, 240
screen = pygame.display.set_mode(size)

replay_buffer = ReplayBuffer(obs_dim=obs_dim, act_dim=act_dim,
                             goal_dim=goal_dim, size=10000)


o_dict, ep_ret, ep_len = env.reset(), 0, 0

o = o_dict["observation"]
dg = o_dict["desired_goal"]


def get_action():
    action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    for i in range(len(action)):
        if pygame.key.get_pressed()[KEY_PAIRS[i][0]]:
            action[i] = 1
        elif pygame.key.get_pressed()[KEY_PAIRS[i][1]]:
            action[i] = -1

    return action


def save_episode(ep_len):
    fp = "data/{}.hdf5".format(int(time.time()))
    size = replay_buffer.size

    with h5py.File(fp, "w") as f:
         obs = f.create_dataset("obs", core.combined_shape(size, obs_dim), dtype='f')
         obs2 = f.create_dataset("obs2", core.combined_shape(size, obs_dim), dtype='f')
         act = f.create_dataset("act", core.combined_shape(size, act_dim), dtype='f')
         rew = f.create_dataset("rew", (size, ), dtype='f')
         done = f.create_dataset("done", (size, ), dtype='f')
         dgoal = f.create_dataset("dgoal", core.combined_shape(size, goal_dim), dtype='f')
         agoal = f.create_dataset("agoal", core.combined_shape(size, goal_dim), dtype='f')

         obs[...] = replay_buffer.obs_buf[:size]
         obs2[...] = replay_buffer.obs2_buf[:size]
         act[...] = replay_buffer.act_buf[:size]
         rew[...] = replay_buffer.rew_buf[:size]
         done[...] = replay_buffer.done_buf[:size]
         dgoal[...] = replay_buffer.dgoal_buf[:size]
         agoal[...] = replay_buffer.agoal_buf[:size]


while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    if pygame.key.get_pressed()[pygame.K_SPACE]:
        save_episode(ep_len)
        replay_buffer = ReplayBuffer(obs_dim=obs_dim, act_dim=act_dim,
                            goal_dim=goal_dim, size=10000)

        o_dict, ep_ret, ep_len = env.reset(), 0, 0
        o = o_dict["observation"]
        dg = o_dict["desired_goal"]

    a = get_action()

    o2_dict, r, d, i = env.step(a)
    ep_ret += r
    ep_len += 1
    o2 = o2_dict["observation"]
    dg2 = o2_dict["desired_goal"]
    ag2 = o2_dict["achieved_goal"]

    replay_buffer.store(o, a, r, o2, d, dg, ag2, i)

    o = o2
    dg = dg2
