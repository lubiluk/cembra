import numpy as np

class FrameBuffer:
    def __init__(self, width=640, height=480, n_frames=4, n_channels=1):
        self.n_channels = n_channels
        self.obs_shape = [n_channels * n_frames, height, width]
        self.framebuffer = np.zeros(self.obs_shape, 'float32')

    def reset(self):
        self.framebuffer = np.zeros_like(self.framebuffer)

        return self.framebuffer

    def update(self, img):
        offset = self.n_channels
        axis = 0
        cropped_framebuffer = self.framebuffer[:-offset]
        self.framebuffer = np.concatenate(
            [img, cropped_framebuffer], axis=axis)

        return self.framebuffer