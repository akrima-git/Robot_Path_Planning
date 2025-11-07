from collections import deque
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time
import json
import random


# Reads map from excel and assigns it to a 2D array

with open("MapAssignment.json") as file:
    config = json.load(file)



width = config["map_width"]
height = config["map_height"]
prob = config["obstacle_probability"]
border = config["border_walls"]
seed = config.get("seed", None)


Sy = random.randint(1,width-2)
Sx = random.randint(1,height-2)




start = (Sy,Sx)

while True:
    Gy = random.randint(1,width-2)
    Gx = random.randint(1,height-2)
    if Gy == Sy and Gx == Sx:
        continue
    if abs(Gy - Sy) <= 3 and abs(Gx - Sx) <= 3:
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
    directions = int(input("4-directional or 8-directional? (4 or 8): "))
    match directions:
        case 4:
            motion = [(-1,0),
                        (0,1),
                        (1,0),
                        (0,-1)]
        case 8:
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

# Function to check if deque is empty
def isEmpty(dq):
    return len(dq) == 0

motion = getMotion()

def traversal(grid, start, goal):


    # Initialise set for visited nodes, and queue for the seen nodes to be visited
    visited = set()
    queue = deque()
    queue.append((start, [start]))
    
    while not isEmpty(queue):
        (y,x), path = queue.popleft()
        if (y,x) == goal:
            return path
        visited.add((y,x))

        for dy, dx in motion:
            ny, nx = y + dy, x + dx

            if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                if grid[ny,nx] == 0 and (ny,nx) not in visited:
                    queue.append(((ny, nx), path + [(ny, nx)]))
    return None

grid = generate_random_map(width,height,prob,border)

map_traversed = grid.copy()
start_time = time.time()
path = traversal(grid, start, goal)
end_time = time.time()

for py,px in path:
    map_traversed[py,px] = 5

map_traversed[Sy,Sx] = 3
map_traversed[Gy, Gx] = 4
print(path)
print("Time elapsed",end_time-start_time,"second(s)")



colours = ["white","darkgrey","grey","red","green","cyan"]

cmap = ListedColormap(colours)
plt.imshow(map_traversed, cmap=cmap)
plt.title("Grid Map (0=free,1=obstacle,3=start,4=goal,5=path)")
plt.show()
