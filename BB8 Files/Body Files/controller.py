import pygame
import time

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Move the Right Stick and Triggers to see their Axis Numbers. Press Ctrl+C to stop.")

try:
    while True:
        pygame.event.pump()
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            if abs(val) > 0.1:
                print(f"Axis {i} value: {val:>6.3f}")
        time.sleep(0.1)
except KeyboardInterrupt:
    pygame.quit()