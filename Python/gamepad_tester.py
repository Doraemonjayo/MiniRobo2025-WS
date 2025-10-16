import pygame
import time

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
if count == 0:
    print("コントローラが接続されていません。")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"使用中のコントローラ: {joystick.get_name()}")
print(f"軸数: {joystick.get_numaxes()}")
print(f"ボタン数: {joystick.get_numbuttons()}")
print(f"ハット数: {joystick.get_numhats()}")

time.sleep(3)

while True:
    pygame.event.pump()

    for i in range(joystick.get_numaxes()):
        print(f"Axis {i}: {joystick.get_axis(i):.3f}", end=" | ")

    for i in range(joystick.get_numbuttons()):
        print(f"Btn {i}: {joystick.get_button(i)}", end=" | ")

    for i in range(joystick.get_numhats()):
        print(f"Hat {i}: {joystick.get_hat(i)}", end=" | ")

    print()
    time.sleep(0.1)  # 出力間隔（調整可能）
