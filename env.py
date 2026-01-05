import numpy as np  
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap    # For visualisation
import time                                     # For measuring execution time
import json

# Import algorithms
from BFS import BFSGraph, BFSTree
from DFS import DFSGraph, DFSTree
from AStar import AStarGraph, AStarTree
from UCS import UCSGraph, UCSTree


with open("MapAssignment.json") as file:    # Load configuration from JSON file
    config = json.load(file)


width = config["map_width"]                 # Map dimensions
height = config["map_height"]
prob = config["obstacle_probability"]       # Obstacle probability
border = config["border_walls"]             # Border walls (boolean)
seed = config.get("seed", None)             # Random seed (optional)
distance = config.get("distance", (width + height) / 4)




def setUpEnv(width, height, obstacle_prob, border=True, seed=None):
    if seed is not None:
        np.random.seed(seed)

    Sy, Sx = np.random.randint(1, height-2), np.random.randint(1, width-2)
    Gy, Gx = np.random.randint(1, height-2), np.random.randint(1, width-2)
    start = (Sy, Sx)

    # Ensure goal is distinct and reasonably far
    while abs(Sy-Gy) + abs(Sx-Gx) < distance:
        Gy, Gx = np.random.randint(1, height-2), np.random.randint(1, width-2)
    goal = (Gy, Gx)

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

    return grid, start, goal

def getMotion(directions):
    if directions == 4:
        return [(-1, 0), (0, 1), (1, 0), (0, -1)] # North, East, South, West
    elif directions == 8:
        return [(-1, 0), (-1, 1), (0, 1), (1, 1), # N, NE, E, SE, S, SW, W, NW
                (1, 0), (1, -1), (0, -1), (-1, -1)]
    return []

if __name__ == "__main__":

    motion = None                   # Get user input for motion model
    while motion is None:
        directions = input("4-directional or 8-directional? (4 or 8): ")
        motion = getMotion(int(directions))
        if not motion:
            print("Invalid choice. Please enter 4 or 8.")
            motion = None

    chosenModel = None                   
    while chosenModel not in ["BFS Graph", "BFS Tree", "DFS Graph", "DFS Tree", "A* Graph", "A* Tree", "UCS Graph", "UCS Tree"]:
        choice = input("Choose a model: \n1. BFS Graph \n2. BFS Tree (Please reduce map size to 20x20 or lower) \n3. DFS Graph\n4. DFS Tree (Please reduce map size to 20x20 or lower) \n5. A* Graph\n6. A* Tree\n7. UCS Graph\n8. UCS Tree\nEnter 1, 2, 3, 4, 5, 6, 7, or 8: ")
        match choice:
            case "1":
                chosenModel = "BFS Graph"
            case "2":
                chosenModel = "BFS Tree"
            case "3":
                chosenModel = "DFS Graph"
            case "4":
                chosenModel = "DFS Tree"
            case "5":
                chosenModel = "A* Graph"
            case "6":
                chosenModel = "A* Tree"
            case "7":
                chosenModel = "UCS Graph"
            case "8":
                chosenModel = "UCS Tree"

            
            case _:
                print("Invalid choice. Please enter 1-8: ")

    grid, start, goal = setUpEnv(width, height, prob, border)# Generate random map














    start_time = time.time()                                  # Start timer
    match chosenModel:
        case "BFS Graph":
            GBFS = BFSGraph(motion)                                  # instantiate BFS class
            path, visited, visitedList = GBFS.traversal(grid, start, goal)
        case "BFS Tree":
            TBFS = BFSTree(motion)
            path, visited, visitedList = TBFS.traversal(grid, start, goal)
        case "DFS Graph":
            GDFS = DFSGraph(motion)                                  # instantiate DFS class
            path, visited, visitedList = GDFS.traversal(grid, start, goal)
        case "DFS Tree":
            TDFS = DFSTree(motion, max_depth =500)                   # set a higher depth limit for larger maps
            path, visited, visitedList = TDFS.traversal(grid, start, goal)
        case "A* Graph":
            GAStar = AStarGraph(motion)                              # instantiate A* class
            path, visited, visitedList = GAStar.traversal(grid, start, goal)
        case  "A* Tree":
            TAStar = AStarTree(motion)  
            path, visited, visitedList = TAStar.traversal(grid, start, goal)
        case  "UCS Graph":
            GUCS = UCSGraph(motion)                                  # instantiate UCS class
            path, visited, visitedList = GUCS.traversal(grid, start, goal)
        case  "UCS Tree":
            TUCS = UCSTree(motion)
            path, visited, visitedList = TUCS.traversal(grid, start, goal)
        case _:
            print("No valid model chosen.")
            path, visited, visitedList = None, set(), []
    end_time = time.time()                                     # End timer 



    # Visualisation

    # 0 = free, 1 = wall, 2 = visited, 3 = start, 4 = goal, 5 = path
    cmap = ListedColormap(["white", "darkgrey", "blue", "red", "green", "cyan"])
    print(path)
    print("Time elapsed", end_time - start_time, "second(s)")

    plt.ion()
    fig, plot = plt.subplots()
    grid[start] = 3                                            # Mark start (red) and goal (green) on grid
    grid[goal] = 4

    if path is None:
        print("No path found")
    else:
        for vy, vx in visitedList:                             # Visualise visited nodes
            if (vy, vx) != start and (vy, vx) != goal:
                grid[vy, vx] = 2                               # Mark visited (blue)
                plot.clear()
                plot.imshow(grid, cmap = cmap, vmin = 0, vmax = 5)
                plt.title(f"Visited tiles ({len(visited)})")
                plt.pause(0.002)

        for py, px in path[1:-1]:                               # Visualise final path
            grid[py, px] = 5                                    # Mark path (cyan)  
            plot.clear()
            plot.imshow(grid, cmap = cmap)
            plt.title(f"Final Path ({len(path)})")
            plt.pause(0.005)


    plt.ioff()
    plt.show()
