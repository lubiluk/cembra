import h5py
import numpy as np
import uuid
import random
import os.path

class BigReplayBuffer():
    """Replay buffer that keeps data on disk so it can get big"""
    def __init__(self, size, obs_size, action_size):
        assert(type(size) == int)
        assert(type(obs_size) == tuple)
        assert(type(action_size) == tuple)

        scratch = '/net/scratch/people/plglubiluk'
        tmp = '/tmp'

        data_dir = scratch if os.path.exists(scratch) else tmp

        self._filename = "{}.hdf5".format(str(uuid.uuid4()))
        self._f = h5py.File(data_dir + '/' + self._filename, "w")

        self._obs_t = self._f.create_dataset("obs_t", (size,) + obs_size, dtype='i')
        self._action = self._f.create_dataset("action", (size,) + action_size, dtype='f')
        self._reward = self._f.create_dataset("reward", (size, 1), dtype='f')
        self._obs_tp1 = self._f.create_dataset("obs_tp1", (size,) + obs_size, dtype='i')
        self._done = self._f.create_dataset("done", (size, 1), dtype='i')

        self._maxsize = size
        self._next_idx = 0
        self._len = 0


    def __len__(self):
        return self._len

    def add(self, obs_t, action, reward, obs_tp1, done):
        self._obs_t[self._next_idx] = obs_t
        self._action[self._next_idx] = action
        self._reward[self._next_idx] = reward
        self._obs_tp1[self._next_idx] = obs_tp1
        self._done[self._next_idx] = done

        self._next_idx = (self._next_idx + 1) % self._maxsize

        if self._len < self._maxsize:
            self._len += 1

    def _encode_sample(self, idxes):
        obses_t, actions, rewards, obses_tp1, dones = [], [], [], [], []
        for i in idxes:
            obses_t.append(self._obs_t[i])
            actions.append(self._action[i])
            rewards.append(self._reward[i])
            obses_tp1.append(self._obs_tp1[i])
            dones.append(self._done[i])
        return (
            np.array(obses_t),
            np.array(actions),
            np.array(rewards),
            np.array(obses_tp1),
            np.array(dones)
        )

    def sample(self, batch_size):
        """Sample a batch of experiences.
        Parameters
        ----------
        batch_size: int
            How many transitions to sample.
        Returns
        -------
        obs_batch: np.array
            batch of observations
        act_batch: np.array
            batch of actions executed given obs_batch
        rew_batch: np.array
            rewards received as results of executing act_batch
        next_obs_batch: np.array
            next set of observations seen after executing act_batch
        done_mask: np.array
            done_mask[i] = 1 if executing act_batch[i] resulted in
            the end of an episode and 0 otherwise.
        """
        idxes = [
            random.randint(0, self._len - 1)
            for _ in range(batch_size)
        ]
        return self._encode_sample(idxes)

# b = BigReplayBuffer(10, (2,2), (1,))
# b.add(np.array([[1, 1], [1, 1]]), np.array([1.1]), 0.1, np.array([[3, 3], [3, 3]]), 0)
# b.add(np.array([[2, 2], [2, 2]]), np.array([1.0]), 0.0, np.array([[4, 4], [4, 4]]), 0)
# print(len(b))
# r = b.sample(10)
# print(r)