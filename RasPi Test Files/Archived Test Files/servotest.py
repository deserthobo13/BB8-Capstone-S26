import time
from adafruit_servokit import ServoKit #needed for controlling the servo driver
import adafruit_bno055 #gyro
import math

from gpiozero import PWMOutputDevice
from gpiozero import OutputDevice

import adafruit_motor
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import busio, board

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # 50Hz for servos

### Testing Control Area ###
DC_drive_motors_test=False
swing_servo_test=False
head_servo_test=False
RestAllServos=False
############################
choice_not_made=True
while choice_not_made:
	test_choice = input("What would you like to test? (Input the corresponding number)\nDC Motors - 1\nSwing Servos - 2\nHead Positioning Servos - 3\nNone, just rest the servos - 4\n")
	
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
#!!USE set_swing!! instead of setting the angle of individual swing_servo motors#
swing_servo = servo.Servo(pca.channels[0]) #use set_swing to not break the opposite servo!!
head_fb = servo.Servo(pca.channels[2]) #front to back CENTER: 105
head_sts = servo.Servo(pca.channels[1]) #side to side CENTER: 95
head_rotate = servo.ContinuousServo(pca.channels[3], min_pulse=810, max_pulse=2300) #self explanatory
swing_servo2 = servo.Servo(pca.channels[4]) #other swing servo. It swings opposite to the first one. USE set_swing
#########################




print("TO CANCEL, USE CTRL+C, DONT CLOSE THE TAB")
print("Waiting 2 secs")
time.sleep(2)
motor_driver_relay.on()
turn_driver_relay.on()

### quality of life servo functions ###
def easy_turn(servo, angle, timeperdegree):
	#This function allows to fine tune the turn rate of a servo
	if angle>int(servo.angle):
		for angle in range(int(servo.angle),angle,1):
			servo.angle=angle
			time.sleep(timeperdegree)
	else:
		for angle in range(int(servo.angle),angle,-1):
			servo.angle=angle
			time.sleep(timeperdegree)
			
def set_swing(degrees): #rolling straight is 90
	#max 125 
	#min 70
	if 125>=degrees>=70:
		difference=degrees-90
		swing_servo.angle = 90 + difference
		swing_servo2.angle = 90 - difference
	else:
		raise ValueError(f"swing: {degrees} requested which is out of range. Range 70-125")
		
def set_head_fb(degrees):
	#center: 105
	#90=105
	#degrees+15 = turn angle
	if 55<=degrees<=125:
		head_fb.angle = degrees+15;
	else:
		raise ValueError(f"head_fb: {degrees} requested which is out of range. Range 55-125")

def set_head_sts(degrees):
	#center: 95
	if 40<=degrees<=140:
		head_sts.angle = degrees+5;
	else:
		raise ValueError(f"head_sts: {degrees} requested which is out of range. Range 40-140")


servo_floppy = [False] * 16  # up to 16 channels

#used for testing to "disable" servos who may be retaining their previous position

def set_servo_floppy(channel, floppy=True):
	
    """Enable or disable floppy mode for a servo channel."""
    global servo_floppy
    if floppy:
        pca.channels[channel].duty_cycle = 0  # stop sending PWM
        servo_floppy[channel] = True
    else:
        # Re-enable by creating a Servo object again
        # (or just keep your old one around)
        servo_floppy[channel] = False

def rest_all_servos():
	set_servo_floppy(0)
	set_servo_floppy(1)
	set_servo_floppy(2)
	set_servo_floppy(3)
	set_servo_floppy(4)

#unused for more traditional "servo.angle = degree" usage
def set_servo_angle(channel, angle):
    """Safely move servo unless it's in floppy mode."""
    if not servo_floppy[channel]:
        s = servo.Servo(pca.channels[channel])
        s.angle = angle
    else:
        print(f"Channel {channel} is in floppy mode; ignoring command.")
#######################################	

def head_turn(degrees):
	#2.65 seconds per turn at speed 1     22.64rpm
	#2.65 seconds per turn at speed -0.92 22.64rpm
	turn_seconds=abs(degrees*(2.5/360))
	if degrees>0:
		head_rotate.throttle=1
		time.sleep(turn_seconds)
		head_rotate.throttle=0
	if degrees<0:
		head_rotate.throttle=-0.92
		time.sleep(turn_seconds)
		head_rotate.throttle=0
	

# Testing is contained in a Try except loop to turn off the DC motor driver 
# when the program is cancelled with ctrl+C. This prevents the motors
# from spinning uncontrollably when the program is cancelled without ctrl+C

# instructions for the DC drive motors
# 0 is full reverse
# 1 is full forward
# 0.5 is neutral
while True:
	try:
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
			# Quick Servo Test swings it back and forth

			set_swing(70) 
			time.sleep(2)
			set_swing(117)
			time.sleep(2)
			
		if head_servo_test:
			# Test Head Servos
			# Set all to neutral position
			#NOTE - Not perfectly 180 degrees
			set_head_fb(90) #65-150
			set_head_sts(90) #45-145
			time.sleep(1)

			# Range test forward-backwards
			set_head_fb(55)
			time.sleep(2)
			set_head_fb(125)
			time.sleep(2)
			set_head_fb(90)
			time.sleep(1)

			# Range test side to side
			set_head_sts(40) 
			time.sleep(2)
			set_head_sts(140)
			time.sleep(2)
			set_head_sts(90)
			time.sleep(1)

			# Accuracy test of continuous head turning motor
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
		
		#this sets the speed of the motors to 0 and turns off the relay.
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

