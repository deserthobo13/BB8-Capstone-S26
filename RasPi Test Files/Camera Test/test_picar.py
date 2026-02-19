from picarx import Picarx
import time

# Initialize the car
px = Picarx()

print("Testing Motors... Wheels should spin FORWARD for 2 seconds.")
px.forward(20)  # 20% speed
time.sleep(2)
px.forward(0)   # Stop
print("Test Complete!")