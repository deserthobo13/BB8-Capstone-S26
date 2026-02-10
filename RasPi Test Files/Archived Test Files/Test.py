import time
import math
import sys

# --- Hardware Libraries ---
from adafruit_servokit import ServoKit
import adafruit_bno055 
from gpiozero import PWMOutputDevice, OutputDevice
import adafruit_motor
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import busio, board
import pygame

# ==========================================
# HARDWARE SETUP
# ==========================================

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50 

DC_motor1 = PWMOutputDevice(13, initial_value=0.5, frequency=10000)
DC_motor2 = PWMOutputDevice(18, initial_value=0.5, frequency=10000)
Turn_motor = PWMOutputDevice(19, initial_value=0.5, frequency=10000)

motor_driver_relay = OutputDevice(17, active_high=False)
turn_driver_relay = OutputDevice(27, active_high=False)

sensor = adafruit_bno055.BNO055_I2C(i2c)

swing_servo = servo.Servo(pca.channels[0]) 
head_fb = servo.Servo(pca.channels[2]) 
head_sts = servo.Servo(pca.channels[1]) 
head_rotate = servo.ContinuousServo(pca.channels[3], min_pulse=810, max_pulse=2300) 
swing_servo2 = servo.Servo(pca.channels[4]) 

servo_floppy = [False] * 16 

# ==========================================
# CONTROLLER SETUP
# ==========================================
pygame.init()
pygame.joystick.init()

def setup_controller():
    if pygame.joystick.get_count() == 0:
        print("\n❌ No Controller found! (Plug in USB or pair Bluetooth)")
        return None
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"\n✅ Controller Connected: {js.get_name()}")
    return js

# ==========================================
# MOTOR FUNCTIONS
# ==========================================

def stop_all_motors():
    DC_motor1.value = 0.5
    DC_motor2.value = 0.5
    Turn_motor.value = 0.5
    head_rotate.throttle = 0
    motor_driver_relay.off()
    turn_driver_relay.off()
    
def enable_motors():
    motor_driver_relay.on()
    turn_driver_relay.on()

def set_swing(degrees): 
    if 125 >= degrees >= 70:
        diff = degrees - 90
        swing_servo.angle = 90 + diff
        swing_servo2.angle = 90 - diff

def set_head_fb(degrees):
    if 55 <= degrees <= 125:
        head_fb.angle = degrees + 15

def set_head_sts(degrees):
    if 40 <= degrees <= 140:
        head_sts.angle = degrees + 5

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# ==========================================
# MAIN PROGRAM LOOP
# ==========================================

running = True

try:
    while running:
        stop_all_motors() # Safety reset
        
        print("\n" + "="*30)
        print("       MAIN MENU")
        print("="*30)
        print("1 - DC Motors Test (Auto)")
        print("2 - Swing Servos Test (Auto)")
        print("3 - Head Servos Test (Auto)")
        print("4 - Rest All Servos")
        print("5 - PS5 Controller Mode")
        print("Q - Quit Program")
        
        choice = input("\nSelect Option: ").strip().lower()

        enable_motors()

        # --- OPTION 5: PS5 CONTROLLER ---
        if choice == '5':
            controller = setup_controller()
            if controller:
                print("\n🎮 PS5 MODE STARTED")
                print("Press [SHARE / CREATE] button to return to Menu.")
                
                in_ps5_mode = True
                while in_ps5_mode:
                    pygame.event.pump()
                    
                    # CHECK EXIT BUTTON (Share/Create is usually Button 8, Options is 9)
                    # Modify '8' if your specific button is different.
                    if controller.get_button(8): 
                        print("🔙 returning to menu...")
                        in_ps5_mode = False
                        time.sleep(1) # Debounce
                        break
                    
                    # 1. Drive (Left Stick Y)
                    axis_ly = controller.get_axis(1)
                    drive_val = (axis_ly * -0.5) + 0.5 
                    DC_motor1.value = drive_val
                    DC_motor2.value = drive_val

                    # 2. Steer (Left Stick X)
                    axis_lx = controller.get_axis(0)
                    turn_val = (axis_lx * 0.5) + 0.5
                    Turn_motor.value = turn_val

                    # 3. Head (Right Stick)
                    axis_ry = controller.get_axis(3)
                    axis_rx = controller.get_axis(2)
                    set_head_fb(map_range(axis_ry, -1, 1, 55, 125))
                    set_head_sts(map_range(axis_rx, -1, 1, 40, 140))

                    # 4. Head Rotate (Triggers)
                    # Check if Triggers are Axes (usually 4/5)
                    trig_l = controller.get_axis(4) 
                    trig_r = controller.get_axis(5)
                    
                    if trig_r > -0.5: 
                        head_rotate.throttle = 1.0
                    elif trig_l > -0.5: 
                        head_rotate.throttle = -1.0
                    else:
                        head_rotate.throttle = 0
                        
                    time.sleep(0.05)

        # --- OPTION 1: DC MOTOR TEST ---
        elif choice == '1':
            print("Running DC Test... (Ctrl+C to cancel)")
            try:
                DC_motor1.value = 0.6; DC_motor2.value = 0.6
                time.sleep(2)
                DC_motor1.value = 0.5; DC_motor2.value = 0.5
                time.sleep(1)
                Turn_motor.value = 1
                time.sleep(1)
                Turn_motor.value = 0.5
            except KeyboardInterrupt:
                print("Test Cancelled.")

        # --- OPTION 2: SWING TEST ---
        elif choice == '2':
            print("Running Swing Test... (Ctrl+C to cancel)")
            try:
                set_swing(70); time.sleep(1)
                set_swing(117); time.sleep(1)
            except KeyboardInterrupt:
                print("Test Cancelled.")

        # --- OPTION 3: HEAD TEST ---
        elif choice == '3':
            print("Running Head Test... (Ctrl+C to cancel)")
            try:
                set_head_fb(55); time.sleep(1)
                set_head_fb(125); time.sleep(1)
                set_head_sts(40); time.sleep(1)
                set_head_sts(140); time.sleep(1)
            except KeyboardInterrupt:
                print("Test Cancelled.")

        # --- OPTION 4: REST ---
        elif choice == '4':
            print("Resting Servos...")
            # Set duty cycle to 0 to stop holding position
            for i in range(5):
                pca.channels[i].duty_cycle = 0

        # --- QUIT ---
        elif choice == 'q':
            print("Exiting.")
            running = False

        else:
            print("Invalid Choice.")

except KeyboardInterrupt:
    print("\n\n🛑 PROGRAM KILLED")

finally:
    stop_all_motors()
    pygame.quit()
    print("Cleaned up and exited.")
