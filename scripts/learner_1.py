#!/usr/bin/env python

RESET_SERVICE = '/cembra/reset'
ACTION_SERVICE = '/cembra/action'

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
import torch
import torch.nn as nn
from cv_bridge import CvBridge, CvBridgeError
from tqdm import trange
from itertools import product

import cembra.srv
import td
import training_utils
from dqn_agent import DQNAgent
from observation_processor import ObservationProcessor
from replay_buffer import ReplayBuffer
from frame_buffer import FrameBuffer


class Learner1:
    def __init__(self):
        rospy.init_node("learner_1")
        rospy.loginfo("Learner 1")

        rospy.wait_for_service(RESET_SERVICE)
        rospy.wait_for_service(ACTION_SERVICE)

        self.reset_proxy = rospy.ServiceProxy(RESET_SERVICE, cembra.srv.Reset)
        self.action_proxy = rospy.ServiceProxy(ACTION_SERVICE, cembra.srv.Action)

        self.observation_processor = ObservationProcessor()
        self.frame_buffer = FrameBuffer(640, 480, 4)

        # Create action conversion talbe
        self.possible_actions = list(product(range(-1,2), repeat=6))
        self.n_actions = len(self.possible_actions)


    def start(self):
        state_shape = self.frame_buffer.obs_shape
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

        (goal, state) = self.reset_env()

        plt.figure(figsize=[12, 18])

        plt.subplot(2, 1, 1)
        plt.title("Goal")
        plt.imshow(goal[0, :, :], cmap='gray')

        plt.subplot(2, 1, 2)
        plt.title("State")
        plt.imshow(state[0, :, :], cmap='gray')

        plt.show()

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
                    [self.reset_env()[1]]
                )
                initial_state_v_history.append(np.max(initial_state_q_values))

                # clear_output(True)py
                print("buffer size = %i, epsilon = %.5f" %
                    (len(exp_replay), agent.epsilon))

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

                plt.show()


        return

    def reset_env(self):
        response = self.reset_proxy()
        goal = self.observation_processor.process(response.goal)
        state = self.observation_processor.process(response.state)
        self.frame_buffer.reset()
        state = self.frame_buffer.update(state)

        return (goal, state)

    def take_action(self, action_index):
        actions = self.possible_actions[action_index]

        response = self.action_proxy(actions)

        state = self.observation_processor.process(response.state)
        state = self.frame_buffer.update(state)

        return (state, response.reward, response.is_done)

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
                _, s = self.reset_env()

        return sum_rewards, s

    def evaluate(self, agent, n_games=1, greedy=False, t_max=10000):
        """ Plays n_games full games. If greedy, picks actions as argmax(qvalues). Returns mean reward. """
        rewards = []
        for _ in range(n_games):
            _, s = self.reset_env()
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
