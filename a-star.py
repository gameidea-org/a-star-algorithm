
# Author: Mujtaba
# Date: Jan 30, 2024

# Description: Game-ready version of A* algorithm in 2D. It only requires start/end points and
# points to avoid (obstacles). Does not create a heavy graph structure explicitly (which is
# memory-inefficient for large game worlds).

import pygame
import heapq
import sys

# Constants for visualization
WIDTH, HEIGHT = 800, 600
CELL_SIZE = 40

def heuristic(node, goal):
    # Euclidean distance heuristic
    return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5

def astar_continuous(start, goal, obstacles=[]):
    unvisited = [(heuristic(start, goal), start)]
    visited = set()
    previous = {start: None}
    cost_so_far = {start: 0}

    while unvisited:
        current_cost, current_node = heapq.heappop(unvisited)

        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = previous[current_node]
            return path[::-1]

        visited.add(current_node)

        neighbors = [
            # adjacent neighbors (left, right, up, down)
            (current_node[0] + 1, current_node[1]),
            (current_node[0] - 1, current_node[1]),
            (current_node[0], current_node[1] + 1),
            (current_node[0], current_node[1] - 1),
            # diagonal neighbors (top-left, top-right, bottom-left, bottom-right)
            (current_node[0] + 1, current_node[1] + 1),
            (current_node[0] - 1, current_node[1] - 1),
            (current_node[0] + 1, current_node[1] - 1),
            (current_node[0] - 1, current_node[1] + 1)
        ]

        for neighbor in neighbors:
            if neighbor not in obstacles:
                new_cost = cost_so_far[current_node] + 1

                if neighbor not in visited or new_cost < cost_so_far.get(neighbor, float('inf')):
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(unvisited, (priority, neighbor))
                    previous[neighbor] = current_node

    return None  # No path found

## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ##
## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ##

## Testing in pygame ##

def draw_grid(screen):
    for x in range(0, WIDTH, CELL_SIZE):
        pygame.draw.line(screen, (255, 255, 255), (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, CELL_SIZE):
        pygame.draw.line(screen, (255, 255, 255), (0, y), (WIDTH, y))

def draw_obstacles(screen, obstacles):
    for obstacle in obstacles:
        pygame.draw.rect(screen, (255, 0, 0), (obstacle[0] * CELL_SIZE, obstacle[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

def draw_path(screen, path):
    for node in path:
        pygame.draw.rect(screen, (0, 255, 0), (node[0] * CELL_SIZE, node[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        pygame.display.flip()
        pygame.time.wait(200)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("A* Algorithm Visualization")

    start_point = (0, 0)
    goal_point = (4, 8)
    obstacles = [(2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (4, 5), (4, 6), (4, 7), (3, 8)]

    path = astar_continuous(start_point, goal_point, obstacles)

    if path:
        print("Path found:", path)
    else:
        print("No path found.")

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        screen.fill((0, 0, 0))
        draw_grid(screen)
        draw_obstacles(screen, obstacles)
        draw_path(screen, path)

        pygame.display.flip()

if __name__ == "__main__":
    main()
