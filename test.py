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
scale = 100 # Scale factor for distances
car_angle = 0  # Initial angle of the car

# Serial communication setup
baud_rate = 9600
ser = serial.Serial()
ser.baudrate = baud_rate
ser.port = "COM3"
ser.open()

# Initialize data storage
angle_data = {angle: {"distances": [], "filtered_ax": [], "filtered_ay": [], "filtered_az": []} for angle in range(0, 360, 10)}

# Function to read data from Arduino
def read_arduino_data():
    try:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8").strip()
            split_line = line.split(",")
            if len(split_line) == 8:
                # Parse data fields
                sensor_angle = int(float(split_line[0]))
                avg_ax = float(split_line[1])
                avg_ay = float(split_line[2])
                avg_az = float(split_line[3])
                distance = float(split_line[4])
                filtered_ax = float(split_line[5])
                filtered_ay = float(split_line[6])
                filtered_az = float(split_line[7])

                # Compute true angle
                true_angle = (car_angle + sensor_angle) % 360

                if true_angle in angle_data:
                    angle_data[true_angle]["distances"].append(distance)
                    angle_data[true_angle]["filtered_ax"].append(filtered_ax)
                    angle_data[true_angle]["filtered_ay"].append(filtered_ay)
                    angle_data[true_angle]["filtered_az"].append(filtered_az)
    except Exception as e:
        print(f"Error reading data: {e}")

# Calculate averages for each angle
def calculate_averages():
    averages = {}
    for angle, data in angle_data.items():
        if data["distances"]:
            avg_distance = sum(data["distances"]) / len(data["distances"])
            avg_filtered_ax = sum(data["filtered_ax"]) / len(data["filtered_ax"]) if data["filtered_ax"] else 0
            avg_filtered_ay = sum(data["filtered_ay"]) / len(data["filtered_ay"]) if data["filtered_ay"] else 0
            avg_filtered_az = sum(data["filtered_az"]) / len(data["filtered_az"]) if data["filtered_az"] else 0
            averages[angle] = (avg_distance, avg_filtered_ax, avg_filtered_ay, avg_filtered_az)
    return averages

# Convert polar coordinates to Cartesian
def convert(angle, distance, center):
    angle_rad = math.radians(angle)
    x = center[0] + distance * scale * math.cos(angle_rad)
    y = center[1] - distance * scale * math.sin(angle_rad)
    return x, y

# Draw a grid for visualization
def draw_grid():
    for x in range(0, WIDTH, 50):
        pygame.draw.line(screen, GRAY, (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, 50):
        pygame.draw.line(screen, GRAY, (0, y), (WIDTH, y), 1)

# Update position based on filtered acceleration
def update_position(position, acceleration, dt):
    position[0] += acceleration[0] * dt * dt / 2  # Integrate acceleration for x
    position[1] += acceleration[1] * dt * dt / 2  # Integrate acceleration for y
def convert_relative(angle, distance, car_position):
    angle_rad = math.radians(angle)
    x = car_position[0] + distance * scale * math.cos(angle_rad)
    y = car_position[1] - distance * scale * math.sin(angle_rad)  # Adjust for Pygame's y-axis
    return x, y
# Main loop
running = True
clock = pygame.time.Clock()

while running:
    dt = clock.tick(60) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Read data from Arduino
    read_arduino_data()

    # Calculate averages
    averages = calculate_averages()

    # Update position using filtered acceleration
    if averages:
        avg_filtered_ax = sum(averages[angle][1] for angle in averages) / len(averages)
        avg_filtered_ay = sum(averages[angle][2] for angle in averages) / len(averages)
        update_position(position, [avg_filtered_ax, avg_filtered_ay], dt)

    # Clear the screen
    screen.fill(BLACK)

    # Draw grid
    draw_grid()

    # Draw the distance map relative to the car's position
    cartesian_points = []
    for angle, (avg_distance, avg_filtered_ax, avg_filtered_ay, avg_filtered_az) in averages.items():
        point = convert_relative(angle, avg_distance, position)  # Use car's current position
        cartesian_points.append(point)
        pygame.draw.circle(screen, RED, (int(point[0]), int(point[1])), 3)

    # Connect the points
    if len(cartesian_points) > 1:
        pygame.draw.polygon(screen, WHITE, cartesian_points, 2)

    # Draw the car's position
    pygame.draw.circle(screen, GREEN, (int(position[0]), int(position[1])), 5)

    # Update the display
    pygame.display.flip()

# Quit Pygame and close the serial port
pygame.quit()
ser.close()