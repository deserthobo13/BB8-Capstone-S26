#relay test
from gpiozero import OutputDevice
import time


relay = OutputDevice(17, active_high=False)
turn_driver_relay = OutputDevice(27, active_high=False)
while True:
	
	# This tests the relay that controls the DC motor driver. I deemed a relay 
	# to be necessary after having a recurring issue of the motor driver allowing
	# the motors to run away.
	
	# I'm sure there was a better fix than adding hardware, but a relay is a fix
	# nonetheless.
	
	relay.on()
	print("Relay on")
	time.sleep(3)
	relay.off()
	print("Relay off")
	time.sleep(3)
	
	turn_driver_relay.on()
	print("Turn Relay on")
	time.sleep(3)

	turn_driver_relay.off()
	print("Turn Relay off")
	time.sleep(3)
	

