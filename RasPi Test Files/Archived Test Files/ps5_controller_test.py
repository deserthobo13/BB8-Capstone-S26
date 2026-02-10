import pygame
import sys

# Initialize Pygame
pygame.init()
pygame.joystick.init()

# Check for controller
if pygame.joystick.get_count() == 0:
    print("❌ No controller found! Please plug in your PS5 controller via USB or pair via Bluetooth.")
    sys.exit()

# Initialize the first controller found
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"✅ Connected to: {joystick.get_name()}")
print(f"🎮 Number of Axes: {joystick.get_numaxes()}")
print(f"🔘 Number of Buttons: {joystick.get_numbuttons()}")
print("------------------------------------------------")
print("Press any button or move joysticks to see inputs...")
print("Press Ctrl+C to quit.")
print("------------------------------------------------")

try:
    while True:
        # Event processing loop
        for event in pygame.event.get():
            
            # --- BUTTON PRESSES ---
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button PRESSED: ID {event.button}")
            
            # --- BUTTON RELEASES ---
            elif event.type == pygame.JOYBUTTONUP:
                # Uncomment the line below if you want to see releases too
                # print(f"Button RELEASED: ID {event.button}")
                pass

            # --- ANALOG STICKS & TRIGGERS (AXES) ---
            elif event.type == pygame.JOYAXISMOTION:
                # Only print if value is significant (avoids stick drift spam)
                if abs(event.value) > 0.1: 
                    print(f"Axis MOVED: ID {event.axis} | Value: {event.value:.2f}")

            # --- D-PAD (HAT) ---
            elif event.type == pygame.JOYHATMOTION:
                print(f"D-Pad (Hat) MOVED: ID {event.hat} | Value: {event.value}")

except KeyboardInterrupt:
    print("\n🛑 Exiting...")
    pygame.quit()
    sys.exit()