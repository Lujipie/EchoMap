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
sensor_position = [400, 400]  
velocity_y = 0.0  
scale = 1  
wall_distance = None  
points = []  

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
    global wall_distance, velocity_y, sensor_position, last_time, points

    try:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8").strip()
            split_line = line.split(",")
            if len(split_line) == 5:  
                # Parse the data
                angle = float(split_line[4])  # Angle in degrees
                wall_distance = float(split_line[3])  # Distance in mm
                acceleration_y = 0  # Acceleration in mm/s^2

                # Reset points when servo goes back to 0°
                if angle == 0:
                    points.clear()

                # Calculate time delta for integration
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # Update velocity and position based on acceleration
                velocity_y += acceleration_y * dt  
                sensor_position[1] += velocity_y * dt * scale  

                # Convert the polar coordinates to Cartesian and store the point
                x = sensor_position[0] + wall_distance * scale * math.cos(math.radians(angle))
                y = sensor_position[1] - wall_distance * scale * math.sin(math.radians(angle))
                if is_valid_point(int(x), int(y)): #if a point is out of range then it wont take the point in an attempt to kill outliers
                    points.append((int(x), int(y)))
    except Exception as e:
        print(f"Error reading data: {e}")
def is_valid_point(x, y, margin=300, smoothing=0.9): 
    if not points:
        return True
    x_coords, y_coords = zip(*points)
    avg_x = sum(x_coords) / len(x_coords)
    avg_y = sum(y_coords) / len(y_coords)
    min_x, max_x = avg_x - margin, avg_x + margin
    min_y, max_y = avg_y - margin, avg_y + margin
    return (min_x <= x <= max_x) and (min_y <= y <= max_y)

# Draw grid for visualization
def draw_grid():
    for x in range(0, WIDTH, 50):
        pygame.draw.line(screen, GRAY, (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, 50):
        pygame.draw.line(screen, GRAY, (0, y), (WIDTH, y), 1)

# Draw all plotted points and lines connecting them
def draw_points_and_lines():
    if len(points) > 1:
        pygame.draw.lines(screen, WHITE, False, points, 2)  # Draw lines between points
    for point in points:
        pygame.draw.circle(screen, RED, point, 3)

# Check if the sensor is close to the wall and make a sound
def check_proximity_to_wall():
    if wall_distance is not None and wall_distance <= 60:  # Within 50mm
        pygame.mixer.Sound.play(pygame.mixer.Sound("beep.wav"))
        print("Proximity alert: Sensor is too close to the wall!")

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

    # Draw all points and connecting lines
    draw_points_and_lines()

    # Draw the sensor position
    pygame.draw.circle(screen, GREEN, (int(sensor_position[0]), int(sensor_position[1])), 5)

    # Check proximity and print alert if close to the wall
    check_proximity_to_wall()

    # Refresh the display
    pygame.display.flip()

# Quit Pygame and close the serial port
pygame.quit()
ser.close()



