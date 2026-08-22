import time
from gpiozero import Robot

print("Initializing motors...")
print("Left Motor: GPIO 17, 27")
print("Right Motor: GPIO 22, 23")

# Initialize L298N motors via gpiozero Robot
# ENA/ENB should be jumpered to 5V on the board
robot = Robot(left=(17, 27), right=(22, 23))

def test_motor(name, duration, forward=True):
    print(f"Testing {name} motor {'forward' if forward else 'backward'}...")
    if name == "left":
        if forward:
            robot.left_motor.forward(0.5)
        else:
            robot.left_motor.backward(0.5)
    else:
        if forward:
            robot.right_motor.forward(0.5)
        else:
            robot.right_motor.backward(0.5)
    
    time.sleep(duration)
    robot.stop()
    time.sleep(1)

try:
    print("--- Starting Motor Test ---")
    test_motor("left", 2, True)
    test_motor("left", 2, False)
    test_motor("right", 2, True)
    test_motor("right", 2, False)
    
    print("Both forward...")
    robot.forward(0.5)
    time.sleep(2)
    robot.stop()
    time.sleep(1)
    
    print("Both backward...")
    robot.backward(0.5)
    time.sleep(2)
    robot.stop()
    
    print("Test complete. Motors stopped.")

except KeyboardInterrupt:
    print("\nTest interrupted. Stopping motors.")
    robot.stop()
