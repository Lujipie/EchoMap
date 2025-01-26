import pygame
import math
import serial
import time

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("RC Car EchoMap")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
GRAY = (200, 200, 200)

# Sensor parameters
sensor_position = [400, 400]  # Sensor position in the center
velocity = [0, 0]  # Velocity (x, y)
position = [400, 400]  # Absolute position (x, y)
scale = 5  # Scale factor for distances
car_angle = 0  # Initial angle of the car (degrees)

# Serial communication setup
baud_rate = 9600
ser = serial.Serial()
ser.baudrate = baud_rate
ser.port = "COM3"
ser.open()

# Persistent data structures
current_position_map = {angle: None for angle in range(0, 360, 2)}  # Map for the current position
car_maps = []  # List to store maps of previous positions
current_sweep_angles = set()  # Track angles in the current sweep

# Time tracking for gyro integration
last_time = time.time()

# Function to read data from Arduino
def read_arduino_data():
    global car_angle, last_time

    try:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8").strip()
            split_line = line.split(",")
            if len(split_line) == 9:
                # Parse the data
                sensor_angle = int(float(split_line[0]))
                vl53_distance = float(split_line[4])
                ultrasonic_distance = float(split_line[8])
                gyro_z = float(split_line[7])  # Gyro Z value (degrees/second)

                # Use ultrasonic distance if VL53L0X distance exceeds 600mm
                if vl53_distance > 600:
                    distance = ultrasonic_distance
                else:
                    distance = vl53_distance

                # Calculate time delta for gyro integration
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # Integrate gyro Z to get the car's facing angle
                car_angle += gyro_z * dt
                car_angle %= 360  # Keep angle within 0-360 degrees

                # Compute true angle for the sensor reading
                true_angle = (car_angle + sensor_angle) % 360
                true_angle = int(true_angle)  # Ensure integer angle

                # Update the average distance for the angle
                if current_position_map[true_angle] is None:
                    current_position_map[true_angle] = distance
                else:
                    # Average old and new distances
                    current_position_map[true_angle] = (
                        current_position_map[true_angle] + distance
                    ) / 2

                # Track the angle in the current sweep
                current_sweep_angles.add(true_angle)
    except Exception as e:
        print(f"Error reading data: {e}")

# Check for completed sweep
def complete_sweep_check():
    global current_sweep_angles, car_maps, current_position_map
    if len(current_sweep_angles) >= 180 / 2:  # Assuming steps of 2 degrees
        current_sweep_angles.clear()
        for angle in current_position_map:
            current_position_map[angle] = None  # Reset distances for the next sweep

# Move to a new position
def move_to_new_position(new_position):
    global car_maps, current_position_map, position

    # Save the current position map to history
    car_maps.append({"position": position[:], "map": current_position_map.copy()})

    # Update the car's position
    position = new_position

    # Reset the current position map
    current_position_map = {angle: None for angle in range(0, 360, 2)}

# Convert polar coordinates to Cartesian relative to origin
def convert_origin(angle, distance):
    angle_rad = math.radians(angle)
    x = sensor_position[0] + distance * scale * math.cos(angle_rad)
    y = sensor_position[1] - distance * scale * math.sin(angle_rad)
    return x, y

# Draw a grid for visualization
def draw_grid():
    for x in range(0, WIDTH, 50):
        pygame.draw.line(screen, GRAY, (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, 50):
        pygame.draw.line(screen, GRAY, (0, y), (WIDTH, y), 1)

# Draw lines connecting points to create a room-like map
def draw_room():
    points = []
    for angle, distance in current_position_map.items():
        if distance is not None:
            x, y = convert_origin(angle, distance)
            points.append((int(x), int(y)))

    if len(points) > 1:
        pygame.draw.lines(screen, WHITE, True, points, 2)  # Connect points with lines (closed loop)

# Update position based on filtered acceleration and scale
def update_position(position, velocity, acceleration, dt, scale):
    velocity[0] += acceleration[0] * dt  # Update velocity for x
    velocity[1] += acceleration[1] * dt  # Update velocity for y
    position[0] += velocity[0] * dt * scale  # Update position for x with scaling
    position[1] += velocity[1] * dt * scale  # Update position for y with scaling

# Main loop
running = True
clock = pygame.time.Clock()

while running:
    dt = clock.tick(60) / 2000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Read data from Arduino
    read_arduino_data()

    # Check for sweep completion
    complete_sweep_check()

    # Clear the screen for redrawing
    screen.fill(BLACK)

    # Draw grid
    draw_grid()

    # Draw persistent points from completed positions
    for car_map in car_maps:
        car_pos = car_map["position"]
        for angle, distance in car_map["map"].items():
            if distance is not None:
                x, y = convert_origin(angle, distance)
                pygame.draw.circle(screen, WHITE, (int(x), int(y)), 2)

    # Draw points for the current position
    for angle, distance in current_position_map.items():
        if distance is not None:
            x, y = convert_origin(angle, distance)
            pygame.draw.circle(screen, RED, (int(x), int(y)), 3)

    # Draw lines to connect points, forming a room-like structure
    draw_room()

    # Draw the car's current position
    pygame.draw.circle(screen, GREEN, (int(position[0]), int(position[1])), 5)

    # Refresh the display
    pygame.display.flip()

# Quit Pygame and close the serial port
pygame.quit()
ser.close()
