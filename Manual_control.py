import pygame
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE, K_a, K_w, K_s, K_d

import numpy as np
import matplotlib.pyplot as plt

import time

import serial

SCREEN_WIDTH = 720
SCREEN_HEIGHT = 480


def wait_for_motors():
    for i in range(5):
        print("Motor enable in:" + str(5-i) + "sec")
        time.sleep(1)

    for i in range(5):
        print("Sequence start in:" + str(5-i) + "sec")
        time.sleep(1)


def run():
    # PYGAME
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    pygame.display.set_caption('GA2DCars')
    pygame.font.init()
    font = pygame.font.SysFont('Arial', 14)
    clock = pygame.time.Clock()
    running = True
    number = 5
    last_number = 0

    # SERIOVKA:
    ser = serial.Serial()
    ser.baudrate = 250000
    ser.port = 'COM3'
    ser.timeout = 0.
    ser.open()

    # wait_for_motors()
    time.sleep(0.8)

    while running:
        if last_number != number:
            last_number = number
            print(last_number)

        data = b'S' + number.to_bytes(1, byteorder='little', signed=True)

        time.sleep(0.0001)
        if number != 5:
            ser.write(data)
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False

        if not keys[pygame.K_w] and not keys[pygame.K_s] and not keys[pygame.K_a] and not keys[pygame.K_d]:
            number = 5

        if keys[pygame.K_r]:
            number = 0

        if keys[pygame.K_w]:
            if keys[pygame.K_a]:
                number = 7
                continue
            if keys[pygame.K_d]:
                number = 9
                continue
            number = 8

        if keys[pygame.K_s]:
            if keys[pygame.K_a]:
                number = 1
                continue
            if keys[pygame.K_d]:
                number = 3
                continue
            number = 2

        if keys[pygame.K_a]:
            number = 4

        if keys[pygame.K_d]:
            number = 6

        if keys[pygame.K_c]:
            number = 10

        if keys[pygame.K_h]:
            number = 11

    pygame.quit()


if __name__ == '__main__':
    run()
