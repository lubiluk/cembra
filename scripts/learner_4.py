#!/usr/bin/env python

import json
import uuid

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
from os.path import expanduser
# Tensorboard has a memory leak, so can't be used for long training
# from torch.utils.tensorboard import SummaryWriter

RESET_SERVICE = '/cembra/reset'
ACTION_SERVICE = '/cembra/action'

INPUT_SIZE = 39

class Actor(nn.Module):
    def __init__(self, num_actions, upper_bound):
        super(Actor, self).__init__()

        self._upper_bound = upper_bound

        self._fc1 = nn.Linear(INPUT_SIZE, 1024)
        self._norm1 = nn.LayerNorm(1024)
        self._fc2 = nn.Linear(1024, 1024)
        self._norm2 = nn.LayerNorm(1024)
        self._fc3 = nn.Linear(1024, 512)
        self._norm3 = nn.LayerNorm(512)
        self._fc4 = nn.Linear(512, num_actions)

        self._fc3.weight.data.uniform_(-0.003, 0.003)
        self._fc3.bias.data.uniform_(-0.003, 0.003)

    def forward(self, inputs):
        x = inputs
        x = self._norm1(F.relu(self._fc1(x)))
        x = self._norm2(F.relu(self._fc2(x)))
        x = self._norm3(F.relu(self._fc3(x)))
        x = torch.tanh(self._fc4(x))
        x = x * self._upper_bound

        return x


class Critic(nn.Module):
    def __init__(self, num_actions):
        super(Critic, self).__init__()

        self._state_fc1 = nn.Linear(INPUT_SIZE, 1024)
        self._state_norm1 = nn.LayerNorm(1024)
        self._state_fc2 = nn.Linear(1024, 1024)
        self._state_norm2 = nn.LayerNorm(1024)
        self._state_fc3 = nn.Linear(1024, 512)
        self._state_norm3 = nn.LayerNorm(512)

        self._action_fc1 = nn.Linear(num_actions, 32)
        self._action_norm1 = nn.LayerNorm(32)

        self._out_fc1 = nn.Linear(544, 1024)
        self._out_norm1 = nn.LayerNorm(1024)
        self._out_fc2 = nn.Linear(1024, 512)
        self._out_norm2 = nn.LayerNorm(512)
        self._out_fc3 = nn.Linear(512, 1)

    def forward(self, inputs):
        mx = inputs[0]
        mx = self._state_norm1(F.relu(self._state_fc1(mx)))
        mx = self._state_norm2(F.relu(self._state_fc2(mx)))
        mx = self._state_norm3(F.relu(self._state_fc3(mx)))

        ax = inputs[1]
        ax = self._action_norm1(F.relu(self._action_fc1(ax)))

        ox = torch.cat((mx, ax), dim=1)
        ox = self._out_norm1(F.relu(self._out_fc1(ox)))
        ox = self._out_norm2(F.relu(self._out_fc2(ox)))
        ox = self._out_fc3(ox)

        return ox


class Learner4:
    def __init__(self):
        rospy.init_node("learner_4")
        rospy.loginfo("Learner 4")

        rospy.wait_for_service(RESET_SERVICE)
        rospy.wait_for_service(ACTION_SERVICE)

        self.reset_proxy = rospy.ServiceProxy(RESET_SERVICE, cembra.srv.Reset4)
        self.action_proxy = rospy.ServiceProxy(ACTION_SERVICE, cembra.srv.Action4)

        self._exp_replay = BigReplayBuffer(20000, (INPUT_SIZE, ), (8,))

        self._n_actions = 8
        self._lower_bound = -1.0
        self._upper_bound = 1.0

        self._device = torch.device(
            'cuda' if torch.cuda.is_available() else 'cpu')

        self._ou_noise = OUActionNoise(
            mean=np.zeros(1), std_deviation=0.2 * np.ones(1))
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

        self._critic_optimizer = optim.Adam(
            self._critic.parameters(), lr=0.002)
        self._actor_optimizer = optim.Adam(self._actor.parameters(), lr=0.001)

        self._total_episodes = 1000
        self._gamma = 0.99
        self._tau = 0.005
        self._batch_size = 64
        self._max_steps = 100

    def start(self):
        ep_reward_history = []
        avg_reward_history = []

        run_id = str(uuid.uuid4())
        json_file = expanduser("~/data/trainings/{}.json".format(run_id))
        actor_file = expanduser("~/data/trainings/{}_actor.pt".format(run_id))
        critic_file = expanduser(
            "~/data/trainings/{}_critic.pt".format(run_id))
        # writer = SummaryWriter(expanduser("~/data/runs/{}".format(run_id)))

        for ep in range(self._total_episodes):
            prev_state = self._reset_env()
            episodic_reward = 0

            for _ in range(self._max_steps):
                torch_prev_state = torch.tensor(
                    prev_state, device=self._device).unsqueeze(dim=0)

                # writer.add_graph(self._actor, torch_prev_state)

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
            # writer.add_scalar('episodic_reward', episodic_reward, ep)

            # Mean of last 40 episodes
            avg_reward = np.mean(ep_reward_history[-40:])
            print("Episode * {} * Avg Reward is ==> {}".format(ep, avg_reward))
            avg_reward_history.append(avg_reward)
            # writer.add_scalar('avg_reward', avg_reward, ep)

            training_data = {
                'avg_reward': avg_reward,
                'avg_reward_history': avg_reward_history
            }

            with open(json_file, 'w') as f:
                json.dump(training_data, f)

            torch.save(self._actor.state_dict(), actor_file)
            torch.save(self._critic.state_dict(), critic_file)

            # writer.close()

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
        legal_action = sampled_actions.clamp(
            self._lower_bound, self._upper_bound)

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

        y = reward_batch + self._gamma * \
            self._target_critic([next_state_batch, target_actions.detach()])

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

        state = self._pose_msg_2_vec(response.object_pose) + \
            self._joint_state_msg_2_vec(response.joint_state) + \
            self._base_state_msg_2_vec(response.base_state) + \
            self._odom_msg_2_vec(response.odom)

        return state

    def _take_action(self, velocities):
        response = self.action_proxy(velocities)

        state = self._pose_msg_2_vec(response.object_pose) + \
            self._joint_state_msg_2_vec(response.joint_state) + \
            self._base_state_msg_2_vec(response.base_state) + \
            self._odom_msg_2_vec(response.odom)

        return (state, response.reward, response.is_done)

    def _joint_state_msg_2_vec(self, msg):
        joint_names = [
            'arm_flex_joint',
            'arm_lift_joint',
            'arm_roll_joint',
            'base_l_drive_wheel_joint',
            'base_r_drive_wheel_joint',
            'base_roll_joint',
            'hand_l_spring_proximal_joint',
            'hand_motor_joint',
            'hand_r_spring_proximal_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
        ]
        return [msg.position[msg.name.index(n)] for n in joint_names]

    def _base_state_msg_2_vec(self, msg):
        joint_names = ['odom_x', 'odom_y', 'odom_t']
        return [msg.actual.positions[msg.joint_names.index(n)] for n in joint_names] + \
            [msg.actual.velocities[msg.joint_names.index(n)] for n in joint_names]

    def _odom_msg_2_vec(self, msg):
        return [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]

    def _pose_msg_2_vec(self, msg):
        return [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]


if __name__ == '__main__':
    try:
        learner = Learner4()
        learner.start()
    except rospy.ROSInterruptException:
        pass
