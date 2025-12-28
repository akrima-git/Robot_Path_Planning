from collections import deque
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time
import json
import random

from BFS import BFSGraph, BFSTree
from AStar import AStarGraph, AStarTree


with open("MapAssignment.json") as file:    # Load configuration from JSON file
    config = json.load(file)


width = config["map_width"]                 # Map dimensions
height = config["map_height"]
prob = config["obstacle_probability"]       # Obstacle probability
border = config["border_walls"]             # Border walls (boolean)
seed = config.get("seed", None)             # Random seed (optional)
distance = config.get("distance", (width + height) / 4)


Sy = random.randint(1, height - 2) if border == True else random.randint(0, height - 1)      # Start position
Sx = random.randint(1, width - 2) if border == True else random.randint(0, width - 1)
start = (Sy, Sx)

while True:                                 # Goal position
    Gy = random.randint(1, height - 2)
    Gx = random.randint(1, width - 2)
    if Gy == Sy and Gx == Sx:               # Ensure goal is not the same as start
        continue
    if abs(Gy - Sy) <= distance and abs(Gx - Sx) <= distance:   # Ensure goal is at least 'distance' away from start
        continue
    else:
        goal = (Gy, Gx)                     # Set goal position
        break

def generate_random_map(width, height, obstacle_prob, border=True, seed=None):
    if seed is not None:
        np.random.seed(seed)

    grid = np.zeros((height, width), dtype=int)


    for y in range(height):                 # Fill grid with obstacles based on probability
        for x in range(width):
            if (y, x) == start or (y, x) == goal:
                continue
            if np.random.rand() < obstacle_prob:
                grid[y, x] = 1
    
    if border:                              # Add border walls if specified  
        grid[0, :] = 1
        grid[-1, :] = 1
        grid[:, 0] = 1
        grid[:, -1] = 1

    return grid


motion = None                   # Get user input for motion model
while motion is None:
    directions = input("4-directional or 8-directional? (4 or 8): ")
    match directions:
        case "4":               # North, East, South, West
            motion = [(-1, 0),
                        (0, 1),
                        (1, 0),
                        (0, -1)]
        case "8":
            motion = [(-1, 0),   # N, NE, E, SE, S, SW, W, NW
                        (-1, 1),
                        (0, 1),
                        (1, 1),
                        (1, 0),
                        (1, -1),
                        (0, -1),
                        (-1, -1)]
        case _:
            print("Enter 4 or 8: ")


chosenModel = None                   
while chosenModel not in ["BFS Graph", "BFS Tree", "A* Graph", "A* Tree"]:
    choice = input("Choose a model: \n1. BFS Graph \n2. BFS Tree \n3. A* Graph \n4. A* Tree\nEnter 1, 2, or 3: ").strip()
    if choice == "1":
        chosenModel = "BFS Graph"
    elif choice == "2":
        chosenModel = "BFS Tree"
    elif choice == "3":
        chosenModel = "A* Graph"
    elif choice == "4":
        chosenModel = "A* Tree"
    else:
        print("Invalid choice. Please enter 1, 2, or 3.")





GBFS = BFSGraph(motion)                                       # instantiate BFS class
TBFS = BFSTree(motion, max_depth=15)

GAStar = AStarGraph(motion)                              # instantiate A* class
TAStar = AStarTree(motion)  

grid = generate_random_map(width, height, prob, border) # Generate random map


start_time = time.time()                                  # Start timer
if chosenModel == "BFS Graph":
    path, visited, visitedList = GBFS.traversal(grid, start, goal)
elif chosenModel == "BFS Tree":
    path, visited, visitedList = TBFS.traversal(grid, start, goal)
elif chosenModel == "A* Graph":
    path, visited, visitedList = GAStar.traversal(grid, start, goal)
elif chosenModel == "A* Tree":
    path, visited, visitedList = TAStar.traversal(grid, start, goal)
else:
    print("No valid model chosen.")
    path, visited, visitedList = None, set(), []
end_time = time.time()                                  # End timer 

# Visualisation

# 0 = free, 1 = wall, 2 = visited, 3 = start, 4 = goal, 5 = path
cmap = ListedColormap(["white", "darkgrey", "blue", "red", "green", "cyan"])
print(path)
print("Time elapsed", end_time - start_time, "second(s)")

plt.ion()
fig, ax = plt.subplots()
grid[start] = 3
grid[goal] = 4

if path is None:
    print("No path found")
else:
    for vy, vx in visitedList:
        if (vy, vx) != start and (vy, vx) != goal:
            grid[vy, vx] = 2
            ax.clear()
            ax.imshow(grid, cmap=cmap, vmin=0, vmax=5)
            plt.title(f"Visited tiles ({len(visited)})")
            plt.pause(0.1)

    for py, px in path[1:-1]:
        grid[py, px] = 5
        ax.clear()
        ax.imshow(grid, cmap=cmap)
        plt.title(f"Final Path ({len(path)})")
        plt.pause(0.2)


plt.ioff()
plt.show()
