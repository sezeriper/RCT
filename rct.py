import math
import sys

import pygame

# --- Constants ---
WIDTH, HEIGHT = 1280, 720
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
        self.start_pos = (100, HEIGHT // 2)
        self.start_angle = 0
        self.target_pos = (WIDTH - 100, HEIGHT // 2)
        self.target_angle = 0
        self.obstacles = [
            CircleObstacle(300, HEIGHT // 2, 80),
            CircleObstacle(WIDTH // 2, HEIGHT // 2, 120),
            CircleObstacle(WIDTH - 300, HEIGHT // 2, 80),
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
                hits.append(
                    {"obs": obs, "t_min": min(valid_ts), "t_max": max(valid_ts)}
                )
    return hits


def find_unintersected_lines(start_p, target_p, obstacles):
    lines = []

    def is_valid(p1, p2):
        hits = get_segment_intersections(p1, p2, obstacles)
        return len(hits) == 0

    if is_valid(start_p, target_p):
        lines.append((start_p, target_p))

    for obs in obstacles:
        tangents_start = get_circle_tangents(start_p, obs)
        for tp in tangents_start:
            if is_valid(start_p, tp):
                lines.append((start_p, tp))

        tangents_target = get_circle_tangents(target_p, obs)
        for tp in tangents_target:
            if is_valid(tp, target_p):
                lines.append((tp, target_p))

    for i in range(len(obstacles)):
        for j in range(i + 1, len(obstacles)):
            common = get_common_tangents(obstacles[i], obstacles[j])
            for p1, p2 in common:
                if is_valid(p1, p2):
                    lines.append((p1, p2))

    return lines


def find_shortest_path_dfs(lines, start_p, target_p, obstacles):
    import math
    from collections import defaultdict

    def round_pt(pt):
        return (round(pt.x, 2), round(pt.y, 2))

    graph = defaultdict(list)
    nodes = {}

    for p1, p2 in lines:
        rp1, rp2 = round_pt(p1), round_pt(p2)
        dist = (p1 - p2).length()
        nodes[rp1] = p1
        nodes[rp2] = p2
        if rp1 != rp2:
            # Check if this edge is already in the graph to avoid duplicates
            if not any(n_key == rp2 for n_key, _, _, _, _ in graph[rp1]):
                graph[rp1].append((rp2, dist, p1, p2, None))
                graph[rp2].append((rp1, dist, p2, p1, None))

    nodes_list = list(nodes.items())
    for i in range(len(nodes_list)):
        for j in range(i + 1, len(nodes_list)):
            rk1, p1 = nodes_list[i]
            rk2, p2 = nodes_list[j]
            if rk1 != rk2:
                # Check if they are on the same obstacle
                for obs in obstacles:
                    d1 = (p1 - obs.pos).length()
                    d2 = (p2 - obs.pos).length()
                    if abs(d1 - obs.radius) < 0.1 and abs(d2 - obs.radius) < 0.1:
                        v1 = (p1 - obs.pos).normalize()
                        v2 = (p2 - obs.pos).normalize()
                        dot = max(-1.0, min(1.0, v1.dot(v2)))
                        angle = math.acos(dot)
                        dist = angle * obs.radius
                        if not any(n_key == rk2 for n_key, _, _, _, _ in graph[rk1]):
                            graph[rk1].append((rk2, dist, p1, p2, obs))
                            graph[rk2].append((rk1, dist, p2, p1, obs))
                        break

    start_key = round_pt(start_p)
    target_key = round_pt(target_p)

    if start_key not in nodes:
        if len(lines) == 1:
            return [(start_p, target_p, None)]
        return None

    shortest_path = None
    min_dist = float("inf")
    best_dist = {}

    def dfs(current_key, current_dist, current_path, visited):
        nonlocal shortest_path, min_dist

        if current_dist >= min_dist:
            return

        # Prune branches that are worse than previously found paths to this node
        if current_key in best_dist and current_dist >= best_dist[current_key]:
            return
        best_dist[current_key] = current_dist

        if current_key == target_key:
            min_dist = current_dist
            shortest_path = list(current_path)
            return

        for next_key, dist, orig_p1, orig_p2, obs in sorted(
            graph[current_key], key=lambda x: x[1]
        ):
            if next_key not in visited:
                visited.add(next_key)
                current_path.append((orig_p1, orig_p2, obs))
                dfs(next_key, current_dist + dist, current_path, visited)
                current_path.pop()
                visited.remove(next_key)

    dfs(start_key, 0, [], {start_key})
    return shortest_path


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

    # 6. DFS Shortest Path
    shortest_path = find_shortest_path_dfs(lines, start_p, target_p, state.obstacles)
    if shortest_path:
        import math

        for p1, p2, obs in shortest_path:
            if obs:
                # Draw arc along the obstacle
                v1 = p1 - obs.pos
                v2 = p2 - obs.pos
                angle1 = math.atan2(v1.y, v1.x)
                angle2 = math.atan2(v2.y, v2.x)
                diff = (angle2 - angle1) % (2 * math.pi)
                if diff > math.pi:
                    diff -= 2 * math.pi
                steps = max(2, int(abs(diff) * obs.radius / 5))
                points = []
                for i in range(steps + 1):
                    t = i / steps
                    a = angle1 + diff * t
                    points.append(
                        obs.pos + pygame.Vector2(math.cos(a), math.sin(a)) * obs.radius
                    )
                if len(points) > 1:
                    pygame.draw.lines(screen, (255, 215, 0), False, points, 4)
            else:
                pygame.draw.line(screen, (255, 215, 0), p1, p2, 4)


def main():
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
    virtual_screen = pygame.Surface((WIDTH, HEIGHT))
    pygame.display.set_caption("Ray Circle Intersection")
    clock = pygame.time.Clock()

    state = State()

    running = True
    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.MOUSEMOTION:
                win_w, win_h = screen.get_size()
                mapped_pos = (
                    event.pos[0] * (WIDTH / win_w),
                    event.pos[1] * (HEIGHT / win_h),
                )
                state.start_pos = mapped_pos
                # Update start angle to face the target
                dx = state.target_pos[0] - state.start_pos[0]
                dy = state.target_pos[1] - state.start_pos[1]
                state.start_angle = math.degrees(math.atan2(dy, dx))
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Map physical mouse position to virtual screen resolution
                win_w, win_h = screen.get_size()
                mapped_pos = (
                    event.pos[0] * (WIDTH / win_w),
                    event.pos[1] * (HEIGHT / win_h),
                )

                if event.button == 1:  # Left click
                    state.target_pos = mapped_pos

                # Update start angle to face the target
                dx = state.target_pos[0] - state.start_pos[0]
                dy = state.target_pos[1] - state.start_pos[1]
                state.start_angle = math.degrees(math.atan2(dy, dx))

        # Draw the current state to the virtual screen
        draw_state(virtual_screen, state)

        # Scale the virtual screen to the actual screen size and blit
        scaled_screen = pygame.transform.scale(virtual_screen, screen.get_size())
        screen.blit(scaled_screen, (0, 0))

        # Update the display
        pygame.display.flip()

        # Maintain frame rate
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
