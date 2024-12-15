# Import necessary Webots libraries
from controller import Robot

# Define constants
TIME_STEP = 64  # Simulation time step in ms
MAX_SPEED = 6.28  # Maximum motor speed for e-puck
IDLE_LIMIT = 50  # Time steps before entering energy-saving mode
OBSTACLE_TIMEOUT = 20  # Time steps before stopping to avoid collision
LIGHT_FOLLOW_LIMIT = 100  # Time steps for light-following behavior

# Initialize robot
robot = Robot()

# Enable proximity sensors
prox_sensors = []
prox_sensor_names = [
    "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"
]
for sensor_name in prox_sensor_names:
    sensor = robot.getDevice(sensor_name)
    sensor.enable(TIME_STEP)
    prox_sensors.append(sensor)

# Enable light sensors
light_sensors = []
light_sensor_names = [
    "ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"
]
for sensor_name in light_sensor_names:
    sensor = robot.getDevice(sensor_name)
    sensor.enable(TIME_STEP)
    light_sensors.append(sensor)

# Initialize motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  # Set motor to velocity control mode
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Function to check for obstacles
def detect_obstacle():
    threshold = 80  # Sensor threshold for obstacle detection
    for i in range(8):
        if prox_sensors[i].getValue() > threshold:
            return True  # Obstacle detected
    return False

# Function to check light intensity
def detect_light():
    light_threshold = 300  # Threshold for light detection
    for i in range(8):
        if light_sensors[i].getValue() > light_threshold:
            return True  # Light detected
    return False

# Main control loop
idle_counter = 0  # Counter for idle state
obstacle_counter = 0  # Counter for prolonged obstacle detection
light_follow_counter = 0  # Counter for light-following duration

while robot.step(TIME_STEP) != -1:
    # Default motor speed
    left_speed = MAX_SPEED / 2
    right_speed = MAX_SPEED / 2

    # Check for obstacles
    if detect_obstacle():
        print("Obstacle detected! Turning...")
        obstacle_counter += 1
        # Simple obstacle avoidance: turn right
        left_speed = MAX_SPEED / 2
        right_speed = -MAX_SPEED / 2

        # Stop after prolonged detection to prevent collisions
        if obstacle_counter > OBSTACLE_TIMEOUT:
            print("Prolonged obstacle detected! Stopping to avoid collision.")
            left_speed = 0
            right_speed = 0
    else:
        obstacle_counter = 0  # Reset counter if no obstacle

    # Check for light
    if detect_light():
        print("Light detected! Moving towards light...")
        light_follow_counter += 1
        # Simple light-following behavior
        left_speed = MAX_SPEED / 1.5
        right_speed = MAX_SPEED / 1.5

        # Stop light-following after a limit
        if light_follow_counter > LIGHT_FOLLOW_LIMIT:
            print("Light-following limit reached! Stopping to avoid over-pursuit.")
            left_speed = 0
            right_speed = 0
    else:
        light_follow_counter = 0  # Reset counter if no light

    # Energy-saving behavior
    if not detect_obstacle() and not detect_light():
        idle_counter += 1
        print(f"Idle state... ({idle_counter} time steps)")
        if idle_counter > IDLE_LIMIT:
            print("Idle for too long. Stopping motors to save energy.")
            left_speed = 0
            right_speed = 0
    else:
        idle_counter = 0  # Reset idle counter if active

    # Set motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
