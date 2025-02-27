import pygame
import numpy as np
import math
import random
from typing import List, Tuple

# Constants
WIDTH, HEIGHT = 1000, 800
G = 6.67430e-2  # Adjusted gravitational constant
BASE_TIMESTEP = 1  # Base timestep (smaller for accuracy)
MIN_CAM_DIST = 300
MAX_CAM_DIST = 3000
ZOOM_SPEED = 15
ROTATION_SPEED = 0.02
SUN_MASS = 1.989e5  # Mass of the Sun

# Precompute constants
ORBIT_LENGTH = 500
ASTEROID_COUNT = 200
ASTEROID_INNER_RADIUS = 220
ASTEROID_OUTER_RADIUS = 280

# Celestial body data (name, mass, radius, color, distance, axial tilt)
PLANET_DATA = [
    ("Mercury", 0.055, 10, (169, 169, 169), 70, 0),
    ("Venus", 0.815, 15, (255, 165, 0), 100, 177.4),
    ("Earth", 1.0, 16, (0, 0, 255), 140, 23.4),
    ("Mars", 0.107, 12, (255, 0, 0), 200, 25.2),
    ("Jupiter", 317.8, 35, (139, 69, 19), 350, 3.1),
    ("Saturn", 95.2, 30, (210, 180, 140), 450, 26.7)
]

class CelestialBody:
    def __init__(self, name: str, mass: float, radius: int, color: Tuple[int, int, int], pos: np.ndarray, velocity: np.ndarray):
        self.name = name
        self.mass = mass
        self.radius = radius
        self.color = color
        self.position = pos.astype(np.float64)
        self.velocity = velocity.astype(np.float64)
        self.orbit = np.full((ORBIT_LENGTH, 2), None)  # Initialize with None
        self.orbit_index = 0

    def update_orbit(self, screen_pos: Tuple[int, int]):
        """Update the orbit trail with a new point."""
        self.orbit[self.orbit_index] = screen_pos
        self.orbit_index = (self.orbit_index + 1) % ORBIT_LENGTH

    def get_orbit_points(self) -> List[Tuple[int, int]]:
        """Get the orbit points in the correct order for rendering, skipping None values."""
        rolled_orbit = np.roll(self.orbit, -self.orbit_index, axis=0)
        return [tuple(point) for point in rolled_orbit if point[0] is not None]

class Camera:
    def __init__(self):
        self.angle_x = math.pi / 4  # Tilt angle
        self.angle_y = -math.pi / 4  # Rotation angle
        self.distance = 1000
        self.target = np.zeros(3, dtype=np.float64)

    def world_to_screen(self, point: np.ndarray) -> Tuple[int, int]:
        """Convert 3D world coordinates to 2D screen coordinates."""
        # Rotation matrices
        cos_y, sin_y = math.cos(self.angle_y), math.sin(self.angle_y)
        cos_x, sin_x = math.cos(self.angle_x), math.sin(self.angle_x)

        # Rotate around Y axis
        x_rot = point[0] * cos_y - point[2] * sin_y
        z_rot = point[0] * sin_y + point[2] * cos_y

        # Rotate around X axis
        y_rot = point[1] * cos_x - z_rot * sin_x
        z_rot = point[1] * sin_x + z_rot * cos_x

        # Perspective projection
        z = z_rot + self.distance
        if z <= 0:  # Behind camera
            return (None, None)

        scale = self.distance / z
        x = int(x_rot * scale + WIDTH / 2)
        y = int(-y_rot * scale + HEIGHT / 2)
        return (x, y)

def create_asteroid_belt(num_asteroids: int, inner_radius: float, outer_radius: float) -> List[CelestialBody]:
    """Create a belt of asteroids."""
    asteroids = []
    for _ in range(num_asteroids):
        angle = random.uniform(0, 2 * math.pi)
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
            np.array([x, y, z]), np.array([vx, 0, vz])
        ))
    return asteroids

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Fixed Solar System Simulator")
    font = pygame.font.SysFont("Arial", 16)
    clock = pygame.time.Clock()

    # Initialize celestial bodies
    bodies = []
    sun = CelestialBody("Sun", SUN_MASS, 30, (255, 255, 0), np.zeros(3), np.zeros(3))
    bodies.append(sun)

    for name, mass, radius, color, distance, _ in PLANET_DATA:
        orbital_speed = math.sqrt(G * SUN_MASS / distance)
        bodies.append(CelestialBody(
            name, mass, radius, color,
            np.array([distance, 0, 0]), np.array([0, 0, orbital_speed])
        ))

    # Add asteroid belt
    bodies += create_asteroid_belt(ASTEROID_COUNT, ASTEROID_INNER_RADIUS, ASTEROID_OUTER_RADIUS)

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
            if event.type == pygame.MOUSEWHEEL:
                camera.distance = np.clip(
                    camera.distance - event.y * ZOOM_SPEED,
                    MIN_CAM_DIST, MAX_CAM_DIST
                )
            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                camera.angle_y -= ROTATION_SPEED
            if keys[pygame.K_RIGHT]:
                camera.angle_y += ROTATION_SPEED
            if keys[pygame.K_UP]:
                camera.angle_x = np.clip(camera.angle_x + ROTATION_SPEED, -math.pi / 2, math.pi / 2)
            if keys[pygame.K_DOWN]:
                camera.angle_x = np.clip(camera.angle_x - ROTATION_SPEED, -math.pi / 2, math.pi / 2)
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
                # Calculate gravitational forces
                dx = sun.position - body.position
                r = np.linalg.norm(dx)
                if r == 0:
                    continue
                force = G * body.mass * sun.mass / r**2
                acceleration = force * dx / (body.mass * r)
                body.velocity += acceleration * substep_dt
                body.position += body.velocity * substep_dt

        # Update orbits
        if show_orbits:
            for body in bodies[1:]:
                screen_pos = camera.world_to_screen(body.position)
                if screen_pos[0] is not None:
                    body.update_orbit(screen_pos)

        # Drawing
        screen.fill((0, 0, 0))

        # Draw orbits
        if show_orbits:
            for body in bodies[1:]:
                orbit_points = body.get_orbit_points()
                if len(orbit_points) >= 2:
                    if body.name == "Asteroid":
                        # Draw asteroid orbits with reduced opacity
                        color = (*body.color, 80)  # Add alpha for transparency
                        pygame.draw.lines(screen, color, False, orbit_points, 1)
                    else:
                        pygame.draw.lines(screen, body.color, False, orbit_points, 1)

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
            screen.blit(surf, (10, 10 + i * 20))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
