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
BLUE = (0, 0, 255)

# Sensor parameters
sensor_position = [400, 400]  # Sensor position in the center
velocity_y = 0.0  # Initial velocity in the y-direction
scale = 1  # Scale factor for distances
wall_distance = None  # Distance to the wall in mm

# Serial communication setup
baud_rate = 9600
ser = serial.Serial()
ser.baudrate = baud_rate
ser.port = "COM3"
ser.open()

# Time tracking for integration
last_time = time.time()

# Function to read data from Arduino
def read_arduino_data():
    global wall_distance, velocity_y, sensor_position, last_time

    try:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8").strip()
            split_line = line.split(",")
            if len(split_line) == 5:  # Expecting [distance, acceleration_y]
                # Parse the data
                wall_distance = float(split_line[3])  # Distance in mm
                acceleration_y = 0  # Acceleration in mm/s^2

                # Calculate time delta for integration
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # Update velocity and position based on acceleration
                velocity_y += acceleration_y * dt  # Integrate acceleration to get velocity
                sensor_position[1] += velocity_y * dt * scale  # Integrate velocity to get position
    except Exception as e:
        print(f"Error reading data: {e}")

# Convert wall distance to Cartesian relative to sensor position
def convert_origin(distance):
    x = sensor_position[0]
    y = sensor_position[1] - distance * scale
    return x, y

# Draw grid for visualization
def draw_grid():
    for x in range(0, WIDTH, 50):
        pygame.draw.line(screen, GRAY, (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, 50):
        pygame.draw.line(screen, GRAY, (0, y), (WIDTH, y), 1)

# Draw the wall as a horizontal line
def draw_wall():
    if wall_distance is not None:
        x, y = convert_origin(wall_distance)
        pygame.draw.line(screen, BLUE, (x - 50, y), (x + 50, y), 3)

# Check if the sensor is close to the wall and make a sound
def check_proximity_to_wall():
    if wall_distance is not None and wall_distance <= 50:  # Within 50mm
        #pygame.mixer.Sound.play(pygame.mixer.Sound("beep.wav"))
        print("hi")

# Main loop
running = True
clock = pygame.time.Clock()

while running:
    dt = clock.tick(60) / 1000.0  # Convert to seconds

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Read data from Arduino
    read_arduino_data()

    # Clear the screen for redrawing
    screen.fill(BLACK)

    # Draw grid
    draw_grid()

    # Draw the wall
    draw_wall()

    # Draw the sensor position
    pygame.draw.circle(screen, GREEN, (int(sensor_position[0]), int(sensor_position[1])), 5)

    # Check proximity and play sound if close to the wall
    check_proximity_to_wall()

    # Refresh the display
    pygame.display.flip()

# Quit Pygame and close the serial port
pygame.quit()
ser.close()
