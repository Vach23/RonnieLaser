import pygame
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE, K_a, K_w, K_s, K_d, K_h, K_s, K_e, K_d

import time

import serial

from random import randint
from copy import copy

SCREEN_WIDTH = 720
SCREEN_HEIGHT = 480
MAX = 600

enable = b'L\0\0\0\0\0\0\n'
disable = b'L1\0\0\0\0\0\n'
home = b'H\0\0\0\0\0\0\n'

motor_on = b'M\0\0\0\0\0\0\n'
motor_off = b'M1\0\0\0\0\0\n'


def run():
    # PYGAME
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    pygame.display.set_caption('Manual_control')
    pygame.font.init()
    font = pygame.font.SysFont('Arial', 14)
    clock = pygame.time.Clock()
    running = True

    data = b'N1234567'
    new_data = b'N1234567'

    # SERIOVKA:
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = '/dev/ttyTHS1'
    ser.timeout = 0.5
    ser.open()

    terminal = True

    if terminal:
        while running:
            i = input()

            if i == 'e':
                ser.write(enable)
                continue
            if i == 'd':
                ser.write(disable)
                continue
            if i == 'h':
                ser.write(home)
                continue
            if i == "on":
                ser.write(motor_on)
                continue
            if i == "off":
                ser.write(motor_off)
                continue
            if i == "r":
                cx = randint(-MAX,MAX)   
                cy = randint(-MAX,MAX)
                print(cx,cy)
                new_data = b'G' + cx.to_bytes(2, byteorder='little', signed=True) + cy.to_bytes(2, byteorder='little', signed=True) + b'\0\0\n' 
                ser.write(new_data)
                continue
            

    else:
        while running:
            if new_data != data:
                data = copy(new_data)
                print(data)
                ser.write(data)

            time.sleep(0.05)
            keys = pygame.key.get_pressed()
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    running = False

            if not keys[pygame.K_w] and not keys[pygame.K_s] and not keys[pygame.K_a] and not keys[pygame.K_d]:
                number = 5

            if keys[pygame.K_h]:
                new_data = b'H\0\0\0\0\0\0\n'
            
            if keys[pygame.K_e]:
                new_data = b'L\0\0\0\0\0\0\n'

            if keys[pygame.K_d]:
                new_data = b'L1\0\0\0\0\0\n'
            
            if keys[pygame.K_r]:
                cx = randint(-MAX,MAX) 
                cy = randint(-MAX,MAX)
                print(cx,cy)
                new_data = b'G' + cx.to_bytes(2, byteorder='little', signed=True) + cy.to_bytes(2, byteorder='little', signed=True) + b'\0\0\n' 
                print(len(new_data))            
            
            if keys[pygame.K_s]:
                cx = randint(-MAX,MAX) 
                cy = randint(-MAX,MAX)
                s = randint(10,50)
                new_data = b'S' + cx.to_bytes(2, byteorder='little', signed=True) + cy.to_bytes(2, byteorder='little', signed=True) + s.to_bytes(2, byteorder='little', signed=True) + b'\n'


    pygame.quit()


if __name__ == '__main__':
    run()
