import csv
import heapq
import math



# Read nodes.csv


def read_nodes(filename):
    nodes = {}
    heuristic = {}

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue

            node_id = int(row[0])
            x = float(row[1])
            y = float(row[2])
            h = float(row[3])

            nodes[node_id] = (x, y)
            heuristic[node_id] = h

    return nodes, heuristic



# Read obstacles.csv


def read_obstacles(filename):
    obstacles = []

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue

            x = float(row[0])
            y = float(row[1])
            r = float(row[2]) / 2.0  # diameter → radius
            obstacles.append((x, y, r))

    return obstacles


# Check if line segment intersects a circle

def edge_intersects_obstacle(p1, p2, obstacle):
    ox, oy, r = obstacle
    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1

    if dx == 0 and dy == 0:
        return False

    t = ((ox - x1) * dx + (oy - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    dist = math.sqrt((closest_x - ox) ** 2 + (closest_y - oy) ** 2)
    return dist <= r



# Read edges.csv AND remove edges that hit obstacles

def read_edges(filename, nodes, obstacles):
    graph = {}

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue

            i = int(row[0])
            j = int(row[1])
            cost = float(row[2])

            p1 = nodes[i]
            p2 = nodes[j]

            # Check collision with all obstacles
            collision = False
            for obs in obstacles:
                if edge_intersects_obstacle(p1, p2, obs):
                    collision = True
                    break

            if collision:
                continue  # skip this edge

            if i not in graph:
                graph[i] = []
            if j not in graph:
                graph[j] = []

            graph[i].append((j, cost))
            graph[j].append((i, cost))

    return graph



# A* Search

def astar(graph, heuristic, start, goal):
    open_set = []
    heapq.heappush(open_set, (heuristic[start], start))

    came_from = {}
    g_cost = {start: 0}
    closed_set = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        if current in closed_set:
            continue
        closed_set.add(current)

        for neighbor, cost in graph.get(current, []):
            tentative_g = g_cost[current] + cost

            if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                g_cost[neighbor] = tentative_g
                f_cost = tentative_g + heuristic[neighbor]
                heapq.heappush(open_set, (f_cost, neighbor))
                came_from[neighbor] = current

    return [start]



# Reconstruct path

def reconstruct_path(came_from, start, goal):
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path



# Write path.csv

def write_path(filename, path):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(path)


# Main

if __name__ == "__main__":
    nodes_file = "nodes.csv"
    edges_file = "edges.csv"
    obstacles_file = "obstacles.csv"
    path_file = "path.csv"

    nodes, heuristic = read_nodes(nodes_file)
    obstacles = read_obstacles(obstacles_file)
    graph = read_edges(edges_file, nodes, obstacles)

    start_node = 1
    goal_node = max(nodes.keys())

    path = astar(graph, heuristic, start_node, goal_node)
    write_path(path_file, path)

    print("Path written to path.csv:")
    print(path)


