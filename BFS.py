from collections import deque
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time
import json
import random

with open("MapAssignment.json") as file:
    config = json.load(file)


width = config["map_width"]
height = config["map_height"]
prob = config["obstacle_probability"]
border = config["border_walls"]
seed = config.get("seed", None)
distance = config.get("distance",(width + height)/4)

Sy = random.randint(1,height-2) if border == True else random.randint(0,height-1)
Sx = random.randint(1,width-2) if border == True else random.randint(0,width-1)
start = (Sy,Sx)
while True:
    Gy = random.randint(1,width-2)
    Gx = random.randint(1,height-2)
    if Gy == Sy and Gx == Sx:
        continue
    if abs(Gy - Sy) <= distance and abs(Gx - Sx) <= distance:
        continue
    else:
        goal = (Gy, Gx)
        break

def generate_random_map(width, height, obstacle_prob, border=True, seed=None):
    if seed is not None:
        np.random.seed(seed)

    grid = np.zeros((height, width), dtype=int)


    for y in range(height):
        for x in range(width):
            if (y,x) == start or (y,x) == goal:
                continue
            if np.random.rand() < obstacle_prob:
                grid[y,x] = 1
    
    if border:
        grid[0, :] = 1
        grid[-1, :] = 1
        grid[:, 0] = 1
        grid[:, -1] = 1

    return grid


# Up, right, down, left

def getMotion():
    directions = (input("4-directional or 8-directional? (4 or 8): "))
    match directions:
        case "4":
            motion = [(-1,0),
                        (0,1),
                        (1,0),
                        (0,-1)]
        case "8":
            motion = [(-1,0),
                        (-1,1),
                        (0,1),
                        (1,1),
                        (1,0),
                        (1,-1),
                        (0,-1),
                        (-1,-1)]
        case _:
            print("Enter 4 or 8")
            return getMotion()
    return motion

def traversal(grid, start, goal):

    motion = getMotion()
    # Initialise set for visited nodes, and queue for the seen nodes to be visited
    visitedList = []
    visited = set()
    queue = deque()
    queue.append((start, [start]))
    
    while queue:
        (y,x), path = queue.popleft()
        if (y,x) == goal:
            return path, visited, visitedList

        for dy, dx in motion:
            ny, nx = y + dy, x + dx

            if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                if grid[ny,nx] == 0 and (ny,nx) not in visited:
                    visitedList.append((ny,nx))
                    visited.add((ny,nx))
                    queue.append(((ny, nx), path + [(ny, nx)]))

    return None, visited, visitedList

grid = generate_random_map(width,height,prob,border)
start_time = time.time()
path, visited, visitedList = traversal(grid, start, goal)
end_time = time.time()

# 0 = free, 1 = wall, 2 = visited, 3 = start, 4 = goal, 5 = path
cmap = ListedColormap(["white","darkgrey","blue","red","green","cyan"])
print(path)
print("Time elapsed",end_time-start_time,"second(s)")



plt.ion()
fig, ax = plt.subplots()
grid[start] = 3
grid[goal] = 4

if path is None:
    print("No path found")
else:
    for vy, vx in visitedList:
        if (vy,vx) != start and (vy,vx) != goal:
            grid[vy,vx] = 2
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



