import gym
import gym_hsr_gazebo
import sys
import pygame

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

pygame.init()

size = width, height = 320, 240
screen = pygame.display.set_mode(size)

env.reset()

while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    for i in range(len(action)):
        if pygame.key.get_pressed()[KEY_PAIRS[i][0]]:
            action[i] = 1
        elif pygame.key.get_pressed()[KEY_PAIRS[i][1]]:
            action[i] = -1

    env.step(action)
