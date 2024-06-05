import pygame
import sys
import math
import numpy as np
import serial
import threading
import queue
import time



# Initialize Pygame
pygame.init()

pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
font = pygame.font.SysFont('arial', 25, bold = pygame.font.Font.bold)

# Screen dimensions
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("2D Racing Simulator")

# Load assets
car_image = pygame.image.load("car2_edit.png")
track_image = pygame.image.load("track.png")

# Load track mask
mask_image = pygame.image.load("track_mask.png")
mask_image = pygame.transform.scale(mask_image, (SCREEN_WIDTH, SCREEN_HEIGHT))


# Scale images
car_image = pygame.transform.scale(car_image, (25, 25))
track_image = pygame.transform.scale(track_image, (SCREEN_WIDTH, SCREEN_HEIGHT))

# Rotate car image by 180 degrees
car_image = pygame.transform.rotate(car_image, 180)

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

SIZE = 0.02
ENGINE_POWER = 40000.0 * 2
WHEEL_MOMENT_OF_INERTIA = 0.5
FRICTION_LIMIT = 400.0

start_time = time.time()  # Initialize timer on start

class Wheel:
    def __init__(self, position, radius):
        self.position = position
        self.init_position = position
        self.gas = 0
        self.brake = 0
        self.steer = 0
        self.wheel_rad = 0.5
        self.radius = radius
        self.linearVelocity = np.array([0, 0])
        self.omega = 0
        self.angle = 0

    def GetWorldVector(self, local_vector):
        angle = self.angle
        return np.array([
            local_vector[0] * math.cos(angle) - local_vector[1] * math.sin(angle),
            local_vector[0] * math.sin(angle) + local_vector[1] * math.cos(angle),
        ])
    
    def GetWorldVectorCarAngle(self, local_vector, car_angle):
        angle = car_angle
        return np.array([
            local_vector[0] * math.cos(angle) - local_vector[1] * math.sin(angle),
            local_vector[0] * math.sin(angle) + local_vector[1] * math.cos(angle),
        ])

    def calculate_force_torque(self, origin, force, car_angle):
        r = origin
        car_angle = -car_angle
        r = np.array([
            r[0] * math.cos(car_angle) - r[1] * math.sin(car_angle),
            r[0] * math.sin(car_angle) + r[1] * math.cos(car_angle),
        ])
        torque = np.cross(r, force)
        force = np.array([
            force[0] * math.cos(car_angle) - force[1] * math.sin(car_angle),
            force[0] * math.sin(car_angle) + force[1] * math.cos(car_angle),
        ])
        return force, torque
    
class Car:
    def __init__(self, angle, x, y):
        self.speed = 0
        self.angle = angle
        self.omega = 0
        self.position = pygame.Vector2(x, y)
        self.velocity = pygame.Vector2(0, 0)
        self.wheel_rad = 0.5
        self.linearVelocity = pygame.Vector2(0, 0)
        self.wheel_base = 12
        self.wheel_track = 14
        self.steering_torque = 0
        self.wheels = [
            Wheel(np.array([-self.wheel_base / 2, self.wheel_track / 2]), self.wheel_rad),
            Wheel(np.array([self.wheel_base / 2, self.wheel_track / 2]), self.wheel_rad),
            Wheel(np.array([-self.wheel_base / 2, -self.wheel_track / 2]), self.wheel_rad),
            Wheel(np.array([self.wheel_base / 2, -self.wheel_track / 2]), self.wheel_rad),
        ]
        self.mass = 5
        self.inertia = 5

    def gas(self, gas):

        gas = np.interp(gas, (-.03, .05), (0, 1))

        gas = np.clip(gas, 0, 1)
        for w in self.wheels[2:4]:
            diff = gas - w.gas
            if diff > 0.1:
                diff = 0.1
            w.gas += diff

    def calc_steering_torque(self):

        t_max = 200
        t_min = -200

        torque = np.clip(self.steering_torque, t_min, t_max)
        
        t_out_max = 1
        t_out_min = -1

        return np.interp(torque, (t_min, t_max), (t_out_min, t_out_max))
    
    def is_off_track(self, mask_image):
        # Get the car's position in the mask image
        car_x = int(self.position.x)
        car_y = int(SCREEN_HEIGHT - self.position.y)
        
        # Check if the position is within the image bounds
        if 0 <= car_x < SCREEN_WIDTH and 0 <= car_y < SCREEN_HEIGHT:
            # Get the pixel value at the car's position
            pixel = mask_image.get_at((car_x, car_y))
            # Check if the pixel is black (on the track)
            if pixel == BLACK:
                return False
        return True

    def brake(self, b):
        for w in self.wheels:
            w.brake = b

    def steer(self, s):
        self.wheels[0].steer = -s*1.2
        self.wheels[1].steer = -s*1.2

        # limit the steering angle to 60 degrees
        for w in self.wheels:
            w.steer = np.clip(w.steer, -math.radians(60), math.radians(60))

    def step(self):
        dt = 1/60/4
        total_force = np.array([0.0, 0.0])
        total_torque = 0.0

        for w in self.wheels:
            dir = np.sign(w.steer - w.angle)
            val = abs(w.steer - w.angle)
            w.angle += (dir * min(50.0 * val, 3.0)) * dt

            forw = w.GetWorldVectorCarAngle((0, 1), -self.angle - w.angle)
            side = w.GetWorldVectorCarAngle((1, 0), -self.angle - w.angle)
            v = self.velocity
            vf = forw[0] * v[0] + forw[1] * v[1]
            vs = side[0] * v[0] + side[1] * v[1]
            v_ang = self.omega * w.init_position[1]
            forw_w = w.GetWorldVectorCarAngle((0, 1), -w.angle)
            side_w = w.GetWorldVectorCarAngle((1, 0), -w.angle)
            vf_w = forw_w[0] * v_ang
            vs_w = side_w[0] * v_ang
            vf += vf_w
            vs += vs_w

            # w.omega += dt * ENGINE_POWER * w.gas / WHEEL_MOMENT_OF_INERTIA / (abs(w.omega) + 5.0)
            w.omega = w.gas*250

            if w.brake >= 0.9:
                w.omega = 0
            elif w.brake > 0:
                BRAKE_FORCE = 15
                dir = -np.sign(w.omega)
                val = BRAKE_FORCE * w.brake
                if abs(val) > abs(w.omega):
                    val = abs(w.omega)
                w.omega += dir * val

            vr = w.omega * w.wheel_rad
            f_force = -vf + vr
            p_force = -vs * 2.5
            f_force *= 1.0
            p_force *= 1.0
            self.steering_torque = p_force*2  # adjust the torque multiplier as needed


            force = np.sqrt(np.square(f_force) + np.square(p_force))
            friction_limit = FRICTION_LIMIT

            # Adjust friction limit based on the car's position
            if self.is_off_track(mask_image):
                friction_limit *= 0.5
                # print('off track')

            if abs(force) > friction_limit:
                f_force /= force
                p_force /= force
                force = friction_limit
                f_force *= force
                p_force *= force

            w.omega -= dt * f_force * w.wheel_rad / WHEEL_MOMENT_OF_INERTIA
            forces = np.array([p_force, f_force])
            ang = -(self.angle + w.angle)
            R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
            force = np.dot(R, forces)
            ang = self.angle
            R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
            force_car = np.dot(R, force)
            torque = force_car[0] * w.init_position[1]
            total_force += force
            total_torque += torque

        acceleration = total_force / self.mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        angular_acceleration = total_torque / self.inertia
        self.omega += angular_acceleration * dt
        self.angle += self.omega * dt

def draw(surface, car):
    rotated_image = pygame.transform.rotate(car_image, -math.degrees(car.angle))
    car_rect = rotated_image.get_rect(center=(car.position[0], SCREEN_HEIGHT - car.position[1]))
    surface.blit(rotated_image, car_rect.topleft)

    count = 0
    for wheel in car.wheels:
        wheel_position = car.position + pygame.Vector2(*wheel.init_position).rotate(-math.degrees(car.angle))
        if count >= 2:
            wheel_image = pygame.Surface((8, 6), pygame.SRCALPHA)
            wheel_image.fill((0, 0, 0, 255))
        else:
            wheel_image = pygame.Surface((5, 6), pygame.SRCALPHA)
            wheel_image.fill((0, 0, 0, 255))
        rotated_wheel_image = pygame.transform.rotate(wheel_image, -math.degrees(car.angle + wheel.angle))
        wheel_rect = rotated_wheel_image.get_rect(center=(wheel_position[0], SCREEN_HEIGHT - wheel_position[1]))
        surface.blit(rotated_wheel_image, wheel_rect.topleft)
        count += 1

    
    # Calculate elapsed time
    elapsed_time = time.time() - start_time

    # Convert time to seconds with one decimal place
    time_text = f"{elapsed_time:.1f} s"
    
    # Render timer text
    time_surface = font.render(time_text, True, BLACK)
    # Position the timer text in the top right corner
    time_rect = time_surface.get_rect(topright=(SCREEN_WIDTH - 10, 10))

    # Draw the timer text on the screen
    surface.blit(time_surface, time_rect)

def draw_torque_visualization(surface, torque):

    if(torque >8):
        torque -=10
    # Parameters for the box
    box_width = 100
    box_height = 20
    max_torque = 1  # Example maximum torque value for scaling

    # Calculate the box height based on the torque value
    torque_height = int(box_height * (abs(torque) / max_torque))
    torque_height = min(torque_height, box_height)  # Ensure it does not exceed the box height

    # Define the position of the box
    box_x = 10
    box_y = SCREEN_HEIGHT - box_height - 10

    # Draw the background box
    pygame.draw.rect(surface, BLACK, (box_x, box_y, box_width, box_height))

    # Draw the torque box (scaling the height based on the torque value)
    if torque < 0:
        # Negative torque, draw red box
        pygame.draw.rect(surface, (255, 0, 0), (box_x, box_y + (box_height - torque_height), box_width, torque_height))
    else:
        # Positive torque, draw green box
        pygame.draw.rect(surface, (0, 255, 0), (box_x, box_y + (box_height - torque_height), box_width, torque_height))


#####communication with arduino###################

# list all available ports
import serial.tools.list_ports

# List all available ports
ports = serial.tools.list_ports.comports()
for port in ports:
    print(port.device)

arduino1 = serial.Serial('/dev/cu.usbserial-AQ02O6YY', 115200, timeout=.1) ###control steering
arduino2 = serial.Serial('/dev/cu.usbserial-AQ02O6YW', 115200, timeout=1) ###control speed
arduino3 = serial.Serial('/dev/cu.usbserial-AQ02O6O7', 115200, timeout=1) ###control speed


data_queue1 = queue.LifoQueue()
data_queue2 = queue.LifoQueue()
def read_from_arduino(arduino, queue):
    while True:
        if arduino.in_waiting > 0:
            try:
                data = arduino.readline().decode('utf-8', errors='ignore').rstrip()
                if validate_data(data):
                    queue.put(data)
                    # print(f"Data received from Arduino: {data}")  # Print statement for logging
            except UnicodeDecodeError:
                continue

def validate_data(data):
    try:
        float(data)
        return True
    except ValueError:
        return False

# Start a separate thread for reading data from Arduino
threading.Thread(target=read_from_arduino, args=(arduino1, data_queue1), daemon=True).start()
threading.Thread(target=read_from_arduino, args=(arduino2, data_queue2), daemon=True).start()

# Main game loop
def main():
    clock = pygame.time.Clock()
    car = Car(0.0, SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
    running = True
    a = [0.0, 0.0, 0.0]
    data2_float = 0.0
    data_received1 = False  # Flag to indicate if data was received
    data_received2 = False  # Flag to indicate if data was receive
    while running:
        #####arduino#########
        while not data_queue1.empty():
            data1 = data_queue1.get()
            try:
                data1_float = float(data1)
                data_received1 = True
                continue
            except ValueError:
                pass  # Ignore invalid data and continue


        while not data_queue2.empty():
            data2 = data_queue2.get()
            try:
                data2_float = float(data2)
                data_received2 = True  # Set flag to true if data is received
                continue
            except ValueError:
                pass  # Ignore invalid data and continue



        if data_received1:
            a[0] = data1_float


        if data_received2:
            if data2_float > -1:
                a[1] = data2_float
            elif data2_float < 0:
                a[2] = +0.8
            else:
                a[1] = 0.0
                a[2] = 0.0


###############Tp is the torque sent to arduino - need modified#################
        # Tp = -50 * a[0] * 0.005 / 30
        force = car.calc_steering_torque() * -3

        # force = 1.59
        # Send Tp to Arduino
    

        # send off track signal to arduino
        # modify below code as needed
        if car.is_off_track(mask_image):
            force = force+10


        arduino1.write(f"{force:.2f}\n".encode())
        arduino3.write(f"{force:.2f}\n".encode())
################################################################################
        global start_time
        # reset car position if off track
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    # Reset car's position to the center of the screen
                    car.position = pygame.Vector2(SCREEN_WIDTH // 2+20, SCREEN_HEIGHT // 2+50)
                    car.velocity = pygame.Vector2(0, 0)
                    car.angle = np.pi/2
                    car.omega = 0
                    start_time = time.time()  # Reset timer
                    for wheel in car.wheels:
                        wheel.omega = 0
        
        
        car.steer(-a[0])
        car.gas(a[1])
        car.brake(a[2])

        for _ in range(4):
            car.step()

        screen.fill(WHITE)
        screen.blit(track_image, (0, 0))
        draw(screen, car)
        draw_torque_visualization(screen, force)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
