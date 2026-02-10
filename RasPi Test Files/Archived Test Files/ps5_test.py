import time
from adafruit_servokit import ServoKit
import adafruit_bno055
import math

from gpiozero import PWMOutputDevice
from gpiozero import OutputDevice

import adafruit_motor
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import busio, board

import evdev
from evdev import InputDevice, categorize, ecodes
import pygame
import sys

# Initialize Pygame
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # 50Hz for servos
current_head_fb = 90 #set head upright

### Testing Control Area ###
DC_drive_motors_test=False
swing_servo_test=False
head_servo_test=False
RestAllServos=False
PS5_control_mode=False
############################
choice_not_made=True
while choice_not_made:
    test_choice = input("What would you like to test? (Input the corresponding number)\nDC Motors - 1\nSwing Servos - 2\nHead Positioning Servos - 3\nNone, just rest the servos - 4\nPS5 Controller Mode - 5\n")
    
    match test_choice:
        case "1":
            DC_drive_motors_test=True
            choice_not_made=False
        case "2":
            swing_servo_test=True
            choice_not_made=False
        case "3":
            head_servo_test=True
            choice_not_made=False
        case "4":
            RestAllServos=True
            choice_not_made=False
        case "5":
            PS5_control_mode=True
            choice_not_made=False
        case _:
            print("\nImproper Number, choose again\n")


### DC Motors and Driver Relay Definitions ###
DC_motor1 = PWMOutputDevice(13, initial_value=0.5, frequency=10000)
DC_motor2 = PWMOutputDevice(18, initial_value=0.5, frequency=10000)
Turn_motor = PWMOutputDevice(19, initial_value=0.5, frequency=10000)
motor_driver_relay = OutputDevice(17, active_high=False)
turn_driver_relay = OutputDevice(27, active_high=False)
##################################

### Gyro Definitions ###
sensor = adafruit_bno055.BNO055_I2C(i2c)
last_val = 0xFFFF
########################

### Servo Definitions ###
swing_servo = servo.Servo(pca.channels[0])
head_fb = servo.Servo(pca.channels[2])
head_sts = servo.Servo(pca.channels[1])
head_rotate = servo.ContinuousServo(pca.channels[3], min_pulse=810, max_pulse=2300)
swing_servo2 = servo.Servo(pca.channels[4])
#########################

print("TO CANCEL, USE CTRL+C, DONT CLOSE THE TAB")
print("Waiting 2 secs")
time.sleep(2)
motor_driver_relay.on()
turn_driver_relay.on()

### quality of life servo functions ###
def easy_turn(servo, angle, timeperdegree):
    if angle>int(servo.angle):
        for angle in range(int(servo.angle),angle,1):
            servo.angle=angle
            time.sleep(timeperdegree)
    else:
        for angle in range(int(servo.angle),angle,-1):
            servo.angle=angle
            time.sleep(timeperdegree)
            
def set_swing(degrees):
    if 70 <= degrees <= 117:
        difference=degrees-90
        swing_servo.angle = 90 + difference
        swing_servo2.angle = 90 - difference
    else:
        print("Swing out of safe range")

def set_head_fb(degrees):
    if 55 <= degrees <= 125:
        head_fb.angle = degrees+15
    else:
        print("Head FB out of safe range")

def set_head_sts(degrees):
    if 40 <= degrees <= 140:
        head_sts.angle = degrees+5
    else:
        print("Head STS out of safe range")

servo_floppy = [False] * 16

def set_servo_floppy(channel, floppy=True):
    global servo_floppy
    if floppy:
        pca.channels[channel].duty_cycle = 0
        servo_floppy[channel] = True
    else:
        servo_floppy[channel] = False

def rest_all_servos():
    set_servo_floppy(0)
    set_servo_floppy(1)
    set_servo_floppy(2)
    set_servo_floppy(3)
    set_servo_floppy(4)

def head_turn(degrees):
    turn_seconds=abs(degrees*(2.5/360))
    if degrees>0:
        head_rotate.throttle=1
        time.sleep(turn_seconds)
        head_rotate.throttle=0
    if degrees<0:
        head_rotate.throttle=-0.92
        time.sleep(turn_seconds)
        head_rotate.throttle=0

def find_ps5():
    devices = [InputDevice(path) for path in evdev.list_devices()]
    for dev in devices:
        if "Wireless Controller" in dev.name or "DualSense" in dev.name:
            print(f"Connected to PS5 controller: {dev.name}")
            return dev
    raise RuntimeError("PS5 controller not found. Pair it first.")

def scale_axis(value):
    if abs(value) < 3000: #Prevents stick drift
        return 0
    return value / 32767 # Max 

while True:
    try:
        if PS5_control_mode:
            #print("Random")
            controller = find_ps5()
            print("PS5 Controller Mode Active")

            for event in pygame.event.get():
                print("in loop")
                 # LEFT STICK VERTICAL → DRIVE (SAFE RANGE)
                if event.type == pygame.JOYAXISMOTION:
                    if abs(event.value) > 0.1: 
                        if event.axis == 1:
                            forward = event.value
                            speed = forward * 0.1  # max ±0.1
                            DC_motor1.value = 0.5 
                            DC_motor2.value = 0.5
                            print("Driving")
                    else:
                        DC_motor1.value = 0.5 
                        DC_motor2.value = 0.5 

                    """
                 # RIGHT STICK HORIZONTAL → TURN MOTOR (SAFE RANGE)
                if event.code == ecodes.ABS_RX:
                    turn = scale_axis(event.value)
                    turn_speed = turn * 0.5
                    Turn_motor.value = turn_speed + 0.5 #range from 0 to 1 on Turn
                    print("Turning")

                    # L2 → SWING LEFT
                if event.code == ecodes.ABS_Z:
                    trigger = event.value / 255
                    set_swing(90 - (trigger * 20))
                    print("Swing Left")

                    # R2 → SWING RIGHT
                if event.code == ecodes.ABS_RZ:
                    trigger = event.value / 255
                    set_swing(90 + (trigger * 20))
                    print("Swing Right")
              
                # D-PAD UP/DOWN → HEAD FB (smooth)
                if event.code == ecodes.ABS_HAT0Y:

                    if event.value == -1:      # UP
                        current_head_fb += 3
                    elif event.value == 1:     # DOWN
                        current_head_fb -= 3
                    
                    current_head_fb = max(55, min(125, current_head_fb)) # Clamp to safe range
                    set_head_fb(current_head_fb)
                    print("Head FB:", current_head_fb)

                if event.type == ecodes.EV_KEY:

                    # Circle → head turn right
                    if event.code == ecodes.BTN_EAST and event.value == 1:
                        head_turn(45)
                        print("Turn head right")

                    # Square → head turn left
                    if event.code == ecodes.BTN_WEST and event.value == 1:
                        head_turn(-45)
                        print("Turn head left")

                    # X → head left
                    if event.code == ecodes.BTN_SOUTH and event.value == 1:
                        set_head_sts(40)
                        print("Set head left")

                    # Triangle → head right
                    if event.code == ecodes.BTN_NORTH and event.value == 1:
                        set_head_sts(140)
                        print("Set head right")
                    """
        ### Your original test modes remain unchanged below ###

        if DC_drive_motors_test:
            time.sleep(2)
            DC_motor1.value = 0.6
            DC_motor2.value = 0.6
            time.sleep(2)
            DC_motor1.value = 0.5
            DC_motor2.value = 0.5
            time.sleep(2)
            DC_motor1.value = 0.4
            DC_motor2.value = 0.4
            time.sleep(2)
            DC_motor1.value = 0.5
            DC_motor2.value = 0.5
            time.sleep(2)
            Turn_motor.value = 1
            time.sleep(2)
            Turn_motor.value = 0.5
            time.sleep(2)
            Turn_motor.value = 0
            time.sleep(2)
            Turn_motor.value = 0.5

        if swing_servo_test:
            set_swing(70)
            time.sleep(2)
            set_swing(117)
            time.sleep(2)

        if head_servo_test:
            set_head_fb(90)
            set_head_sts(90)
            time.sleep(1)
            set_head_fb(55)
            time.sleep(2)
            set_head_fb(125)
            time.sleep(2)
            set_head_fb(90)
            time.sleep(1)
            set_head_sts(40)
            time.sleep(2)
            set_head_sts(140)
            time.sleep(2)
            set_head_sts(90)
            time.sleep(1)
            head_turn(180)
            time.sleep(2)
            head_turn(-180)
            time.sleep(2)
            head_turn(-180)
            time.sleep(2)
            head_turn(180)
            time.sleep(1)
            head_turn(360)
            time.sleep(2)

    except (KeyboardInterrupt, ValueError) as E:
        DC_motor2.value = 0.5
        DC_motor1.value = 0.5
        motor_driver_relay.off()
        turn_driver_relay.off()
        rest_all_servos()

        if type(E) is KeyboardInterrupt:
            print("\nKeyboardInterrupt detected, exiting")
            time.sleep(2)
            exit()
        else:
            print(E)
            print("\nExiting in 10 seconds")
            time.sleep(10)
