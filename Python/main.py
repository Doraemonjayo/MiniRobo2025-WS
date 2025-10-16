import pygame
import time
import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ("192.168.137.5", 8000)

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
if count == 0:
    print("コントローラが接続されていません。")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

lstick = [0, 0]
rstick = [0, 0]
ltrigger = 0
rtrigger = 0

wheels = [0, 0, 0, 0]
arms = [0, 0]

def apply_deadband(value, deadband):
    if abs(value) < deadband:
        value = 0
    return value

while True:
    pygame.event.pump()

    lstick[0] = apply_deadband(joystick.get_axis(0), 0.1)
    lstick[1] = apply_deadband(-joystick.get_axis(1), 0.1)
    rstick[0] = apply_deadband(joystick.get_axis(2), 0.1)
    rstick[1] = apply_deadband(-joystick.get_axis(3), 0.1)
    ltrigger = (joystick.get_axis(4) + 1) / 2
    rtrigger = (joystick.get_axis(5) + 1) / 2

    wheels[0] = int(32767 * (0.4 * lstick[0] + 0.4 * lstick[1] + 0.2 * rstick[0]))  # fl
    wheels[1] = int(32767 * -(-0.4 * lstick[0] + 0.4 * lstick[1] - 0.2 * rstick[0]))  # fr
    wheels[2] = int(32767 * (-0.4 * lstick[0] + 0.4 * lstick[1] + 0.2 * rstick[0]))  # bl
    wheels[3] = int(32767 * -(0.4 * lstick[0] + 0.4 * lstick[1] - 0.2 * rstick[0]))  # br
    arms[0] = ltrigger / 8
    arms[1] = rtrigger / 8

    data = struct.pack("<Bhhhh", 1, wheels[0], wheels[1], wheels[2], wheels[3])
    sock.sendto(data, target)
    data = struct.pack("<Bff", 2, arms[0], arms[1])
    sock.sendto(data, target)

    print(wheels, arms)
    time.sleep(0.01)
