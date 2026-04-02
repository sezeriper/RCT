import math
import sys

import pygame

# --- Constants ---
WIDTH, HEIGHT = 800, 600
FPS = 60

# Colors (RGB)
BLACK = (0, 0, 0)
RED = (255, 50, 50)  # Starting position/direction
BLUE = (50, 150, 255)  # Target position/direction
GRAY = (150, 150, 150)  # Obstacle color


class CircleObstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    @property
    def pos(self):
        return pygame.Vector2(self.x, self.y)


class Ray:
    def __init__(self, origin, direction):
        self.origin = pygame.Vector2(origin)
        self.direction = pygame.Vector2(direction).normalize()

    def point_at(self, t):
        return self.origin + self.direction * t


def ray_circle_intersection(ray, circle):
    """
    Returns a list of distances 't' to all intersection points.
    """
    # Vector from circle center to ray origin
    L = ray.origin - circle.pos

    # Quadratic equation coefficients: at^2 + bt + c = 0
    # a = ray.direction.dot(ray.direction) = 1 (since direction is normalized)
    b = 2 * ray.direction.dot(L)
    c = L.dot(L) - circle.radius**2

    discriminant = b**2 - 4 * c

    if discriminant < 0:
        return []

    if discriminant == 0:
        return [-b / 2]

    t1 = (-b - math.sqrt(discriminant)) / 2
    t2 = (-b + math.sqrt(discriminant)) / 2

    return [t1, t2]


def get_circle_tangents(point, circle):
    """
    Returns the two tangent points on the circle from a given external point.
    """
    point = pygame.Vector2(point)
    center = circle.pos
    d_vec = center - point
    d = d_vec.length()

    if d <= circle.radius:
        return []  # Inside or on the boundary: no external tangents

    # Angle to the center
    angle_to_center = math.atan2(d_vec.y, d_vec.x)
    # Angle between center-line and tangent-line: sin(alpha) = r / d
    alpha = math.asin(circle.radius / d)

    # Length of tangent line: L^2 = d^2 - r^2
    L = math.sqrt(d**2 - circle.radius**2)

    # Tangent points
    t1 = (
        point
        + pygame.Vector2(
            math.cos(angle_to_center - alpha), math.sin(angle_to_center - alpha)
        )
        * L
    )
    t2 = (
        point
        + pygame.Vector2(
            math.cos(angle_to_center + alpha), math.sin(angle_to_center + alpha)
        )
        * L
    )

    return [t1, t2]


def get_common_tangents(c1, c2):
    """
    Returns a list of pairs of points (p1, p2) representing the four common tangents
    between two circles.
    """
    res = []
    d_vec = c2.pos - c1.pos
    d = d_vec.length()

    if d < 0.0001:
        return []

    # Check if one circle is inside another
    if d < abs(c1.radius - c2.radius):
        return []

    angle_c = math.atan2(d_vec.y, d_vec.x)

    # 1. External Tangents
    # cos(phi) = (r1 - r2) / d
    ratio_ext = (c1.radius - c2.radius) / d
    if -1 <= ratio_ext <= 1:
        phi_ext = math.acos(ratio_ext)
        for sign in [-1, 1]:
            a = angle_c + sign * phi_ext
            p1 = c1.pos + pygame.Vector2(math.cos(a), math.sin(a)) * c1.radius
            p2 = c2.pos + pygame.Vector2(math.cos(a), math.sin(a)) * c2.radius
            res.append((p1, p2))

    # 2. Internal Tangents
    # cos(phi) = (r1 + r2) / d
    ratio_int = (c1.radius + c2.radius) / d
    if -1 <= ratio_int <= 1:
        phi_int = math.acos(ratio_int)
        for sign in [-1, 1]:
            a = angle_c + sign * phi_int
            p1 = c1.pos + pygame.Vector2(math.cos(a), math.sin(a)) * c1.radius
            # For internal tangents, the second circle's point is on the opposite side relative to C1
            a_opp = a + math.pi
            p2 = c2.pos + pygame.Vector2(math.cos(a_opp), math.sin(a_opp)) * c2.radius
            res.append((p1, p2))

    return res


class State:
    def __init__(self):
        self.start_pos = (100, 300)
        self.start_angle = 0
        self.target_pos = (700, 300)
        self.target_angle = 0
        self.obstacles = [
            CircleObstacle(250, 300, 40),
            CircleObstacle(400, 300, 60),
            CircleObstacle(550, 300, 40),
        ]


def draw_arrow(surface, color, position, angle_degrees, length=100):
    """
    Draws an arrow to represent position and direction.
    """
    angle = math.radians(angle_degrees)

    # Proportions of the arrow
    shaft_width = 12
    head_width = 36
    head_length = 30

    # Define points of a horizontal arrow pointing to the right, starting at (0,0)
    base_points = [
        (0, -shaft_width / 2),  # Top of shaft base
        (length - head_length, -shaft_width / 2),  # Top of shaft end
        (length - head_length, -head_width / 2),  # Top of arrow head
        (length, 0),  # Arrow tip
        (length - head_length, head_width / 2),  # Bottom of arrow head
        (length - head_length, shaft_width / 2),  # Bottom of shaft end
        (0, shaft_width / 2),  # Bottom of shaft base
    ]

    # Rotate and translate points to the target position and angle
    rotated_points = []
    for x, y in base_points:
        # 2D Rotation matrix logic
        rot_x = x * math.cos(angle) - y * math.sin(angle)
        rot_y = x * math.sin(angle) + y * math.cos(angle)

        # Translate to the actual coordinates on the screen
        final_x = rot_x + position[0]
        final_y = rot_y + position[1]
        rotated_points.append((final_x, final_y))

    pygame.draw.polygon(surface, color, rotated_points)


def get_segment_intersections(p1, p2, obstacles):
    d_vec = p2 - p1
    dist = d_vec.length()
    if dist < 1e-4:
        return []
    ray = Ray(p1, d_vec)
    
    hits = []
    for obs in obstacles:
        L = obs.pos - p1
        t_proj = L.dot(ray.direction)
        t_closest = max(0, min(dist, t_proj))
        closest_pt = p1 + ray.direction * t_closest
        dist_to_center = (closest_pt - obs.pos).length()
        
        if dist_to_center < obs.radius - 0.1: 
            ts = ray_circle_intersection(ray, obs)
            valid_ts = [t for t in ts if -0.1 < t < dist + 0.1]
            if valid_ts:
                hits.append({
                    'obs': obs,
                    't_min': min(valid_ts),
                    't_max': max(valid_ts)
                })
    return hits


def find_unintersected_lines(start_p, target_p, obstacles, depth=0):
    if depth > 10:
        return []
        
    hits = get_segment_intersections(start_p, target_p, obstacles)
    
    if not hits:
        return [(start_p, target_p)]
        
    hits.sort(key=lambda h: h['t_min'])
    first_obs = hits[0]['obs']
    
    hits.sort(key=lambda h: h['t_max'], reverse=True)
    last_obs = hits[0]['obs']
    
    lines = []
    
    if first_obs != last_obs:
        common = get_common_tangents(first_obs, last_obs)
        for p1, p2 in common:
            lines.extend(find_unintersected_lines(p1, p2, obstacles, depth + 1))
            
    tangents_start = get_circle_tangents(start_p, first_obs)
    for tp in tangents_start:
        lines.extend(find_unintersected_lines(start_p, tp, obstacles, depth + 1))
        
    tangents_target = get_circle_tangents(target_p, last_obs)
    for tp in tangents_target:
        lines.extend(find_unintersected_lines(tp, target_p, obstacles, depth + 1))
        
    return lines


def draw_state(screen, state):
    """
    Draws the current state of the environment.
    """
    # 1. Clear the screen
    screen.fill(BLACK)

    # 2. Draw the starting position (Red Arrow)
    draw_arrow(screen, RED, state.start_pos, state.start_angle, length=50)

    # 3. Draw the target position (Blue Arrow)
    draw_arrow(screen, BLUE, state.target_pos, state.target_angle, length=50)

    # 4. Draw obstacles
    for obstacle in state.obstacles:
        pygame.draw.circle(screen, GRAY, obstacle.pos, obstacle.radius)

    # 5. Recursive pathfinding
    start_p = pygame.Vector2(state.start_pos)
    target_p = pygame.Vector2(state.target_pos)
    
    lines = find_unintersected_lines(start_p, target_p, state.obstacles)
    
    for p1, p2 in lines:
        pygame.draw.line(screen, (0, 255, 0), p1, p2, 1)
        pygame.draw.circle(screen, (0, 255, 0), (int(p1.x), int(p1.y)), 3)
        pygame.draw.circle(screen, (0, 255, 0), (int(p2.x), int(p2.y)), 3)


def main():
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Ray Circle Intersection")
    clock = pygame.time.Clock()

    state = State()

    running = True
    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    state.target_pos = event.pos
                elif event.button == 3:  # Right click
                    state.start_pos = event.pos

                # Update start angle to face the target
                dx = state.target_pos[0] - state.start_pos[0]
                dy = state.target_pos[1] - state.start_pos[1]
                state.start_angle = math.degrees(math.atan2(dy, dx))

        # Draw the current state
        draw_state(screen, state)

        # Update the display
        pygame.display.flip()

        # Maintain frame rate
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
