#!/usr/bin/env python

import cembra.srv
import matplotlib.pyplot as plt
import rospy
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from big_replay_buffer import BigReplayBuffer
from frame_buffer import FrameBuffer
from observation_processor import ObservationProcessor
from ou_action_noise import OUActionNoise
from torch_utils import *

RESET_SERVICE = '/cembra/reset'
ACTION_SERVICE = '/cembra/action'

class Actor(nn.Module):
    def __init__(self, num_actions, upper_bound):
        super(Actor, self).__init__()

        self._upper_bound = upper_bound

        self._conv1 = nn.Conv2d(4, 16, kernel_size=3, stride=2, padding=1)
        self._conv2 = nn.Conv2d(16, 16, kernel_size=3, stride=2, padding=1)
        self._conv3 = nn.Conv2d(16, 10, kernel_size=3, stride=2, padding=1)
        self._fc1 = nn.Linear(3000, 512)
        self._norm1 = nn.LayerNorm(512)
        self._fc2 = nn.Linear(512, 512)
        self._norm2 = nn.LayerNorm(512)
        self._fc3 = nn.Linear(512, num_actions)

        self._fc3.weight.data.uniform_(-0.003, 0.003)
        self._fc3.bias.data.uniform_(-0.003, 0.003)

    def forward(self, inputs):
        x = inputs
        x = F.relu(self._conv1(x))
        x = F.relu(self._conv2(x))
        x = F.relu(self._conv3(x))
        x = F.avg_pool2d(x, 4)
        x = x.view(x.size(0), -1)
        x = self._norm1(F.relu(self._fc1(x)))
        x = self._norm2(F.relu(self._fc2(x)))
        x = torch.tanh(self._fc3(x))
        x = x * self._upper_bound

        return x

class Critic(nn.Module):
    def __init__(self, num_actions):
        super(Critic, self).__init__()

        self._state_conv1 = nn.Conv2d(4, 16, kernel_size=3, stride=2, padding=1)
        self._state_conv2 = nn.Conv2d(16, 16, kernel_size=3, stride=2, padding=1)
        self._state_conv3 = nn.Conv2d(16, 10, kernel_size=3, stride=2, padding=1)
        self._state_fc1 = nn.Linear(48000, 16)
        self._state_norm1 = nn.LayerNorm(16)
        self._state_fc2 = nn.Linear(16, 32)
        self._state_norm2 = nn.LayerNorm(32)

        self._action_fc1 = nn.Linear(num_actions, 32)
        self._action_norm1 = nn.LayerNorm(32)

        self._out_fc1 = nn.Linear(64, 512)
        self._out_norm1 = nn.LayerNorm(512)
        self._out_fc2 = nn.Linear(512, 512)
        self._out_norm2 = nn.LayerNorm(512)
        self._out_fc3 = nn.Linear(512, 1)

    def forward(self, inputs):
        mx = inputs[0]
        mx = F.relu(self._state_conv1(mx))
        mx = F.relu(self._state_conv2(mx))
        mx = F.relu(self._state_conv3(mx))
        mx = mx.view(mx.size(0), -1)
        mx = self._state_norm1(F.relu(self._state_fc1(mx)))
        mx = self._state_norm2(F.relu(self._state_fc2(mx)))

        ax = inputs[1]
        ax = self._action_norm1(F.relu(self._action_fc1(ax)))

        ox = torch.cat((mx, ax), dim=1)
        ox = self._out_norm1(F.relu(self._out_fc1(ox)))
        ox = self._out_norm2(F.relu(self._out_fc2(ox)))
        ox = self._out_fc3(ox)

        return ox

class Learner3:
    def __init__(self):
        rospy.init_node("learner_3_ddpg")
        rospy.loginfo("Learner 3 DDPG")

        rospy.wait_for_service(RESET_SERVICE)
        rospy.wait_for_service(ACTION_SERVICE)

        self.reset_proxy = rospy.ServiceProxy(RESET_SERVICE, cembra.srv.Reset3)
        self.action_proxy = rospy.ServiceProxy(ACTION_SERVICE, cembra.srv.Action3)

        self._observation_processor = ObservationProcessor()
        self._frame_buffer = FrameBuffer(640, 480, 4)
        self._exp_replay = BigReplayBuffer(200000, (4, 480, 640), (8,))

        self._n_actions = 8
        self._lower_bound = -1.0
        self._upper_bound = 1.0

        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self._ou_noise = OUActionNoise(mean=np.zeros(1), std_deviation=0.2 * np.ones(1))
        self._actor = Actor(self._n_actions, self._upper_bound)
        self._critic = Critic(self._n_actions)
        self._target_actor = Actor(self._n_actions, self._upper_bound)
        self._target_critic = Critic(self._n_actions)

        self._actor.to(self._device)
        self._critic.to(self._device)
        self._target_actor.to(self._device)
        self._target_critic.to(self._device)

        weight_copy(self._target_actor, self._actor)
        weight_copy(self._target_critic, self._critic)

        self._critic_optimizer = optim.Adam(self._critic.parameters(), lr=0.002)
        self._actor_optimizer = optim.Adam(self._actor.parameters(), lr=0.001)

        self._total_episodes = 200
        self._gamma = 0.99
        self._tau = 0.005
        self._batch_size = 64
        self._max_steps = 1000

    def start(self):
        ep_reward_history = []
        avg_reward_history = []

        for ep in range(self._total_episodes):
            prev_state = self._reset_env()
            episodic_reward = 0

            for _ in range(self._max_steps):
                torch_prev_state = torch.tensor(prev_state, device=self._device).unsqueeze(dim=0)

                action = self._policy(torch_prev_state, self._ou_noise)
                state, reward, done = self._take_action(action)

                self._exp_replay.add(prev_state, action, reward, state, done)
                episodic_reward += reward

                self._learn()

                weight_update(self._target_actor, self._actor, self._tau)
                weight_update(self._target_critic, self._critic, self._tau)

                if done:
                    break

                prev_state = state
            
            ep_reward_history.append(episodic_reward)

             # Mean of last 40 episodes
            avg_reward = np.mean(ep_reward_history[-40:])
            print("Episode * {} * Avg Reward is ==> {}".format(ep, avg_reward))
            avg_reward_history.append(avg_reward)

        # Plotting graph
        # Episodes versus Avg. Rewards
        plt.plot(avg_reward_history)
        plt.xlabel("Episode")
        plt.ylabel("Avg. Epsiodic Reward")
        plt.show()


    def _policy(self, state, noise_object):
        self._actor.eval()
        sampled_actions = self._actor(state).squeeze()
        self._actor.train()

        noise = torch.from_numpy(noise_object()).to(self._device)
        # Adding noise to action
        sampled_actions = sampled_actions + noise

        # We make sure action is within bounds
        legal_action = sampled_actions.clamp(self._lower_bound, self._upper_bound)

        return legal_action.detach().to('cpu').numpy()

    def _learn(self):
        def to_tensor(arr):
            return torch.from_numpy(arr).float().to(self._device)

        record = self._exp_replay.sample(self._batch_size)
        # Convert to tensors
        state_batch, action_batch, reward_batch, next_state_batch, _ = map(
            to_tensor, record
        )

        # Training and updating Actor & Critic networks.
        # See Pseudo Code.
        target_actions = self._target_actor(next_state_batch)

        y = reward_batch + self._gamma * self._target_critic([next_state_batch, target_actions.detach()])
        
        self._critic_optimizer.zero_grad()
        critic_value = self._critic([state_batch, action_batch])
        critic_loss = F.mse_loss(critic_value, y.detach())
        critic_loss.backward()
        self._critic_optimizer.step()

        self._actor_optimizer.zero_grad()
        actions = self._actor(state_batch)
        critic_value = self._critic([state_batch, actions])
        # Used `-value` as we want to maximize the value given
        # by the critic for our actions
        actor_loss = -critic_value.mean()
        actor_loss.backward()
        self._actor_optimizer.step()

    def _reset_env(self):
        response = self.reset_proxy()

        self._frame_buffer.reset()

        state = self._observation_processor.process(response.state)
        state = self._frame_buffer.update(state)

        return state

    def _take_action(self, velocities):
        response = self.action_proxy(velocities)

        state = self._observation_processor.process(response.state)
        state = self._frame_buffer.update(state)

        return (state, response.reward, response.is_done)

if __name__ == '__main__':
    try:
        learner = Learner3()
        learner.start()
    except rospy.ROSInterruptException:
        pass
