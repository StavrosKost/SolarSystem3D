import pygame
import numpy as np
import math
import random

# Constants
WIDTH, HEIGHT = 1000, 800
G = 6.67430e-2  # Adjusted gravitational constant
BASE_TIMESTEP = 1  # Base timestep (smaller for accuracy)
BUTTON_COLOR = (50, 50, 50)
BUTTON_HOVER_COLOR = (100, 100, 100)

# Camera settings
MIN_CAM_DIST = 300
MAX_CAM_DIST = 3000
ZOOM_SPEED = 15
ROTATION_SPEED = 0.02

class CelestialBody:
    def __init__(self, name, mass, radius, color, pos, velocity):
        self.name = name
        self.mass = mass
        self.radius = radius
        self.color = color
        self.position = np.array(pos, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        self.orbit = []

    def calculate_gravity(self, other):
        dx = other.position[0] - self.position[0]
        dy = other.position[1] - self.position[1]
        dz = other.position[2] - self.position[2]
        r = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if r == 0:
            return np.zeros(3)
        
        force = G * self.mass * other.mass / r**2
        theta = math.atan2(dy, dx)
        phi = math.atan2(math.hypot(dx, dy), dz)
        
        return np.array([
            force * math.cos(theta) * math.sin(phi),
            force * math.sin(theta) * math.sin(phi),
            force * math.cos(phi)
        ])

    def update_position(self, bodies, dt):
        total_acceleration = np.zeros(3)
        for body in bodies:
            if body is not self:
                total_acceleration += self.calculate_gravity(body) / self.mass
        
        self.velocity += total_acceleration * dt
        self.position += self.velocity * dt

class Camera:
    def __init__(self):
        self.angle_x = math.pi/4  # Tilt angle
        self.angle_y = -math.pi/4 # Rotation angle
        self.distance = 1000
        self.target = np.zeros(3)

    def world_to_screen(self, point):
        # Rotate around Y axis
        y_rot = np.array([
            [math.cos(self.angle_y), 0, math.sin(self.angle_y)],
            [0, 1, 0],
            [-math.sin(self.angle_y), 0, math.cos(self.angle_y)]
        ])
        
        # Rotate around X axis
        x_rot = np.array([
            [1, 0, 0],
            [0, math.cos(self.angle_x), -math.sin(self.angle_x)],
            [0, math.sin(self.angle_x), math.cos(self.angle_x)]
        ])
        
        # Transform point
        translated = point - self.target
        rotated = y_rot @ x_rot @ translated
        z = rotated[2] + self.distance
        
        # Perspective projection
        if z <= 0:  # Behind camera
            return (None, None)
        
        scale = self.distance / z
        x = int(rotated[0] * scale + WIDTH/2)
        y = int(-rotated[1] * scale + HEIGHT/2)
        return (x, y)

def create_asteroid_belt(num_asteroids, inner_radius, outer_radius):
    asteroids = []
    for _ in range(num_asteroids):
        angle = random.uniform(0, 2*math.pi)
        radius = random.uniform(inner_radius, outer_radius)
        height = random.uniform(-50, 50)
        
        x = radius * math.cos(angle)
        y = height
        z = radius * math.sin(angle)
        
        # Orbital velocity
        speed = math.sqrt(G * SUN_MASS / radius) * random.uniform(0.9, 1.1)
        vx = -speed * math.sin(angle) * random.uniform(0.9, 1.1)
        vz = speed * math.cos(angle) * random.uniform(0.9, 1.1)
        
        asteroids.append(CelestialBody(
            "Asteroid", 0.1, 1, (150, 150, 150),
            [x, y, z], [vx, 0, vz]
        ))
    return asteroids

# Celestial constants
SUN_MASS = 1.989e5
PLANET_DATA = [
    # Name       Mass    Radius  Color           Distance  Axial tilt
    ("Mercury",  0.055,  10,    (169, 169, 169), 70,       0),
    ("Venus",    0.815,  15,    (255, 165, 0),   100,      177.4),
    ("Earth",    1.0,    16,    (0, 0, 255),     140,      23.4),
    ("Mars",     0.107,  12,    (255, 0, 0),     200,      25.2),
    ("Jupiter", 317.8,   35,    (139, 69, 19),   350,      3.1),
    ("Saturn",  95.2,    30,    (210, 180, 140), 450,      26.7)
]

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Stable Solar System Simulator")
    font = pygame.font.SysFont("Arial", 16)
    clock = pygame.time.Clock()

    # Initialize celestial bodies
    bodies = []
    sun = CelestialBody("Sun", SUN_MASS, 30, (255, 255, 0), [0, 0, 0], [0, 0, 0])
    bodies.append(sun)

    for name, mass, radius, color, distance, tilt in PLANET_DATA:
        orbital_speed = math.sqrt(G * SUN_MASS / distance)
        bodies.append(CelestialBody(
            name, mass, radius, color,
            [distance, 0, 0],
            [0, 0, orbital_speed]
        ))

    # Add asteroid belt
    bodies += create_asteroid_belt(200, 220, 280)

    # Initialize camera
    camera = Camera()
    show_orbits = True
    speed_factor = 1.0

    running = True
    while running:
        dt = clock.tick(60) / 1000  # Delta time in seconds
        
        # Handle input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # Mouse wheel zoom
            if event.type == pygame.MOUSEWHEEL:
                camera.distance = np.clip(
                    camera.distance - event.y * ZOOM_SPEED,
                    MIN_CAM_DIST, MAX_CAM_DIST
                )
            
            # Key presses
            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                camera.angle_y -= ROTATION_SPEED
            if keys[pygame.K_RIGHT]:
                camera.angle_y += ROTATION_SPEED
            if keys[pygame.K_UP]:
                camera.angle_x = np.clip(camera.angle_x + ROTATION_SPEED, -math.pi/2, math.pi/2)
            if keys[pygame.K_DOWN]:
                camera.angle_x = np.clip(camera.angle_x - ROTATION_SPEED, -math.pi/2, math.pi/2)
            if keys[pygame.K_o]:
                show_orbits = not show_orbits
            if keys[pygame.K_EQUALS]:
                speed_factor = min(speed_factor * 1.5, 100.0)
            if keys[pygame.K_MINUS]:
                speed_factor = max(speed_factor / 1.5, 0.1)

        # Physics update with substepping
        steps = int(speed_factor) + 1
        substep_dt = BASE_TIMESTEP * speed_factor / steps
        for _ in range(steps):
            for body in bodies[1:]:  # Skip sun
                body.update_position(bodies, substep_dt)

        # Update orbits
        if show_orbits:
            for body in bodies[1:]:
                screen_pos = camera.world_to_screen(body.position)
                if screen_pos[0] is not None:
                    body.orbit.append(screen_pos)
                    if len(body.orbit) > 500:
                        body.orbit.pop(0)

        # Drawing
        screen.fill((0, 0, 0))
        
        # Draw orbits
        if show_orbits:
            for body in bodies[1:]:
                if len(body.orbit) >= 2:
                    pygame.draw.lines(screen, body.color, False, body.orbit, 1)
        
        # Draw bodies
        for body in bodies:
            screen_pos = camera.world_to_screen(body.position)
            if screen_pos[0] is not None:
                pygame.draw.circle(screen, body.color, screen_pos, body.radius)
        
        # UI
        info = [
            f"Speed: {speed_factor:.1f}x",
            f"Camera Dist: {camera.distance:.0f}",
            f"Camera Angles: {math.degrees(camera.angle_x):.1f}°, {math.degrees(camera.angle_y):.1f}°",
            "[+/-] Speed  [Arrows] Rotate  [O] Orbits"
        ]
        for i, text in enumerate(info):
            surf = font.render(text, True, (255, 255, 255))
            screen.blit(surf, (10, 10 + i*20))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
