#!/usr/bin/env python

RESET_SERVICE = '/cembra/reset'
ACTION_SERVICE = '/cembra/action'

import glob
from itertools import product

import cembra.srv
import cv2
import h5py
import matplotlib.pyplot as plt
import numpy as np
import rospy
import sensor_msgs.msg
import torch
import torch.nn as nn
from cv_bridge import CvBridge, CvBridgeError
from ros_numpy import msgify
from tqdm import trange

import td
import training_utils
from dqn_agent import DQNAgent
from frame_buffer import FrameBuffer
from observation_processor import ObservationProcessor
from replay_buffer import ReplayBuffer


class Learner1:
    def __init__(self):
        rospy.init_node("learner_2_dqn")
        rospy.loginfo("Learner 2 DQN")

        rospy.wait_for_service(RESET_SERVICE)
        rospy.wait_for_service(ACTION_SERVICE)

        self.reset_proxy = rospy.ServiceProxy(RESET_SERVICE, cembra.srv.ResetEnv2)
        self.action_proxy = rospy.ServiceProxy(ACTION_SERVICE, cembra.srv.ActionEnv2)

        self.observation_processor = ObservationProcessor()
        self.frame_buffer = FrameBuffer(640, 480, 4)

        # Create action conversion talbe
        self.possible_actions = list(product(range(-1,2), repeat=8))
        self.n_actions = len(self.possible_actions)


    def start(self):
        state_shape = self.get_state_shape()
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        agent = DQNAgent(state_shape, self.n_actions, epsilon=1).to(device)
        target_network = DQNAgent(state_shape, self.n_actions).to(device)
        target_network.load_state_dict(agent.state_dict())

        timesteps_per_epoch = 1
        batch_size = 16
        total_steps = 3 * 10**6
        decay_steps = 10**6

        opt = torch.optim.Adam(agent.parameters(), lr=1e-4)

        init_epsilon = 1
        final_epsilon = 0.1

        loss_freq = 50
        refresh_target_network_freq = 5000
        eval_freq = 5000

        max_grad_norm = 50

        mean_rw_history = []
        td_loss_history = []
        grad_norm_history = []
        initial_state_v_history = []

        exp_replay = ReplayBuffer(10**5)

        self.preload_exp(exp_replay)

        state = self.reset_env()

        plot = rospy.get_param("plot", False)

        if plot:
            plt.figure(figsize=[12, 9])
            plt.title("State")
            plt.imshow(state[0, :, :], cmap='gray')

            plt.show(block=False)

        for step in trange(total_steps + 1):
            if not training_utils.is_enough_ram():
                print('less that 100 Mb RAM available, freezing')
                print('make sure everythin is ok and make KeyboardInterrupt to continue')
                try:
                    while True:
                        pass
                except KeyboardInterrupt:
                    pass

            agent.epsilon = training_utils.linear_decay(init_epsilon, final_epsilon, step, decay_steps)

            # play
            _, state = self.play_and_record(state, agent, exp_replay, timesteps_per_epoch)

            # train
            states, actions, rewards, next_states, is_done = exp_replay.sample(batch_size)

            loss = td.compute_loss(states, actions, rewards, next_states, is_done, agent, target_network, device=device)

            loss.backward()
            grad_norm = nn.utils.clip_grad_norm_(agent.parameters(), max_grad_norm)
            opt.step()
            opt.zero_grad()

            if step % loss_freq == 0:
                td_loss_history.append(loss.data.cpu().item())
                grad_norm_history.append(grad_norm)

            if step % refresh_target_network_freq == 0:
                # Load agent weights into target_network
                target_network.load_state_dict(agent.state_dict())

            if step % eval_freq == 0:
                mean_rw_history.append(self.evaluate(agent, n_games=3, greedy=True))
                initial_state_q_values = agent.get_qvalues(
                    [self.reset_env()]
                )
                initial_state_v_history.append(np.max(initial_state_q_values))

                # clear_output(True)py
                print("buffer size = %i, epsilon = %.5f" %
                    (len(exp_replay), agent.epsilon))

                if plot:
                    plt.figure(figsize=[16, 9])

                    plt.subplot(2, 2, 1)
                    plt.title("Mean reward per life")
                    plt.plot(mean_rw_history)
                    plt.grid()

                    assert not np.isnan(td_loss_history[-1])
                    plt.subplot(2, 2, 2)
                    plt.title("TD loss history (smoothened)")
                    plt.plot(training_utils.smoothen(td_loss_history))
                    plt.grid()

                    plt.subplot(2, 2, 3)
                    plt.title("Initial state V")
                    plt.plot(initial_state_v_history)
                    plt.grid()

                    plt.subplot(2, 2, 4)
                    plt.title("Grad norm history (smoothened)")
                    plt.plot(training_utils.smoothen(grad_norm_history))
                    plt.grid()

                    plt.show(block=False)


        return

    def reset_env(self):
        response = self.reset_proxy()
        state = self.observation_processor.process(response.state)

        self.frame_buffer.reset()

        state = self.frame_buffer.update(state)

        return state

    def take_action(self, action_index):
        actions = self.possible_actions[action_index]

        response = self.action_proxy(actions)

        state = self.observation_processor.process(response.state)
        state = self.frame_buffer.update(state)

        return (state, response.reward, response.is_done)

    def preload_exp(self, exp_replay):
        files = glob.glob("/home/lubiluk/Documents/episodes_environment_2/*.hdf5")

        for f in files:
            with h5py.File(f, 'r') as f:
                size = len(f['actions'])

                for i in range(size):
                    s = msgify(sensor_msgs.msg.Image, f['states'][i], 'bgr8')
                    a = f['actions'][i]
                    r = f['rewards'][i]
                    d = f['is_dones'][i]
                    ns = msgify(sensor_msgs.msg.Image, f['states'][i + 1] if i < size - 1 else f['states'][i], 'bgr8')

                    state = self.observation_processor.process(s)
                    state = self.frame_buffer.update(state)

                    next_state = self.observation_processor.process(ns)
                    next_state = self.frame_buffer.update(next_state) 

                    exp_replay.add(state, a, r, next_state, d)

        

    def get_state_shape(self):
        shape = self.frame_buffer.obs_shape

        return shape

    def play_and_record(self, initial_state, agent, exp_replay, n_steps=1):
        """
        Play the game for exactly n steps, record every (s,a,r,s', done) to replay buffer. 
        Whenever game ends, add record with done=True and reset the game.
        It is guaranteed that env has done=False when passed to this function.

        PLEASE DO NOT RESET ENV UNLESS IT IS "DONE"

        :returns: return sum of rewards over time and the state in which the env stays
        """
        s = initial_state
        sum_rewards = 0

        # Play the game for n_steps as per instructions above
        for _ in range(n_steps):
            qvalues = agent.get_qvalues([s])
            action = agent.sample_actions(qvalues)[0]

            next_s, r, done = self.take_action(action)
            
            exp_replay.add(s, action, r, next_s, done)
            s = next_s

            if done:
                s = self.reset_env()

        return sum_rewards, s

    def evaluate(self, agent, n_games=1, greedy=False, t_max=2000):
        """ Plays n_games full games. If greedy, picks actions as argmax(qvalues). Returns mean reward. """
        rewards = []
        for _ in range(n_games):
            s = self.reset_env()
            reward = 0
            for _ in range(t_max):
                qvalues = agent.get_qvalues([s])
                action = qvalues.argmax(axis=-1)[0] if greedy else agent.sample_actions(qvalues)[0]
                s, r, done = self.take_action(action)
                reward += r
                if done:
                    break

            rewards.append(reward)
        return np.mean(rewards)


if __name__ == '__main__':
    try:
        learner = Learner1()
        learner.start()
    except rospy.ROSInterruptException:
        pass
