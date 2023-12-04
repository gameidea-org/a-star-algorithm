
# Mujtaba
# Date: Dec 4, 2023

# In astar_continuous(), continous refers to the fact that it works on
# continous 2D space (instead of a limited graph)

import heapq

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
            (current_node[0] + 1, current_node[1]),
            (current_node[0] - 1, current_node[1]),
            (current_node[0], current_node[1] + 1),
            (current_node[0], current_node[1] - 1),
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

# Example usage:
start_point = (0, 0)
goal_point = (4, 4)

path = astar_continuous(start_point, goal_point)

if path:
    print("Path found:", path)
else:
    print("No path found.")
