import csv
import random
import math
import heapq

# ===============================
# PARAMETERS (easy to tune)
# ===============================
NUM_SAMPLES = 100        # PRM samples (excluding start & goal)
NEIGHBOR_RADIUS = 0.25   # Connection radius
BOUNDS = 0.5

START = (-0.5, -0.5)
GOAL  = (0.5, 0.5)

# ===============================
# Geometry utilities
# ===============================
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def segment_intersects_circle(p1, p2, circle):
    cx, cy, r = circle
    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1

    if dx == 0 and dy == 0:
        return distance(p1, (cx, cy)) <= r

    t = ((cx-x1)*dx + (cy-y1)*dy) / (dx*dx + dy*dy)
    t = max(0.0, min(1.0, t))

    closest = (x1 + t*dx, y1 + t*dy)
    return distance(closest, (cx, cy)) <= r


def collision_free(p1, p2, obstacles):
    for obs in obstacles:
        if segment_intersects_circle(p1, p2, obs):
            return False
    return True


# ===============================
# Read obstacles.csv
# ===============================
def read_obstacles(filename):
    obstacles = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            x = float(row[0])
            y = float(row[1])
            r = float(row[2]) / 2.0
            obstacles.append((x, y, r))
    return obstacles


# ===============================
# PRM construction
# ===============================
def sample_free(obstacles):
    while True:
        x = random.uniform(-BOUNDS, BOUNDS)
        y = random.uniform(-BOUNDS, BOUNDS)
        if collision_free((x, y), (x, y), obstacles):
            return (x, y)


def build_prm(obstacles):
    nodes = [START, GOAL]

    # Sample nodes
    while len(nodes) < NUM_SAMPLES + 2:
        p = sample_free(obstacles)
        nodes.append(p)

    edges = []

    for i in range(len(nodes)):
        for j in range(i+1, len(nodes)):
            if distance(nodes[i], nodes[j]) <= NEIGHBOR_RADIUS:
                if collision_free(nodes[i], nodes[j], obstacles):
                    edges.append((i+1, j+1, distance(nodes[i], nodes[j])))

    return nodes, edges


# ===============================
# A* search
# ===============================
def astar(nodes, edges):
    graph = {}
    for i, j, cost in edges:
        graph.setdefault(i, []).append((j, cost))
        graph.setdefault(j, []).append((i, cost))

    start = 1
    goal = 2

    open_set = []
    heapq.heappush(open_set, (0, start))

    g = {start: 0}
    parent = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(parent, start, goal)

        for nbr, cost in graph.get(current, []):
            new_g = g[current] + cost
            if nbr not in g or new_g < g[nbr]:
                g[nbr] = new_g
                h = distance(nodes[nbr-1], GOAL)
                heapq.heappush(open_set, (new_g + h, nbr))
                parent[nbr] = current

    return [start]


def reconstruct_path(parent, start, goal):
    path = [goal]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path


# ===============================
# Write CSV files
# ===============================
def write_nodes(nodes):
    with open("nodes.csv", "w", newline="") as f:
        writer = csv.writer(f)
        for i, (x, y) in enumerate(nodes, start=1):
            h = distance((x, y), GOAL)
            writer.writerow([i, x, y, h])


def write_edges(edges):
    with open("edges.csv", "w", newline="") as f:
        writer = csv.writer(f)
        for e in edges:
            writer.writerow(e)


def write_path(path):
    with open("path.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(path)


# ===============================
# Main
# ===============================
if __name__ == "__main__":
    obstacles = read_obstacles("obstacles.csv")

    nodes, edges = build_prm(obstacles)
    path = astar(nodes, edges)

    write_nodes(nodes)
    write_edges(edges)
    write_path(path)

    print("PRM nodes:", len(nodes))
    print("PRM edges:", len(edges))
    print("Path:", path)


