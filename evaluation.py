import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
import json

# Import algorithms
from BFS import BFSGraph, BFSTree
from DFS import DFSGraph, DFSTree
from AStar import AStarGraph, AStarTree
from UCS import UCSGraph, UCSTree
from env import setUpEnv, getMotion


with open("MapAssignment.json") as file:    # Load config from JSON file
    config = json.load(file)


width = config["map_width"]                 # Map dimensions
height = config["map_height"]
prob = config["obstacle_probability"]       # Obstacle probability
border = config["border_walls"]             # Border walls (boolean)
seed = config.get("seed", None)             # Random seed (optional)
distance = config.get("distance", (width + height) / 4)

trials = 5                                  # Number of maps to test per direction mode


results = []

print(f"Starting Evaluation: {trials} trials per mode on {width}x{height} grid")


for direction in [4, 8]:
    print(f"Evaluating {direction}-directional movement")
    motion = getMotion(direction)

    # Instantiate classes
    algos = {
        "BFS Graph": BFSGraph(motion),
        "BFS Tree": BFSTree(motion, max_depth = 15),
        "DFS Graph": DFSGraph(motion),
        "DFS Tree": DFSTree(motion, max_depth = 15),
        "A* Graph": AStarGraph(motion),
        "A* Tree": AStarTree(motion),
        "UCS Graph": UCSGraph(motion),
        "UCS Tree": UCSTree(motion)
    }

    for i in range(trials):
        # Generate one random map for this trial that all algorithms will use for fairness
        grid, start, goal = setUpEnv(width, height, prob)
        
        print(f"Trial {i+1}/{trials}: Start {start}, Goal {goal}")
        
        for name, algorithm in algos.items():
            
            # Run the algorithm and time it
            start_time = time.time()
            path, visited, visitedList = algorithm.traversal(grid, start, goal)
            end_time = time.time()
            
            duration = end_time - start_time
            path_len = len(path) if path else 0 # 0 if no path found
            nodes_visited = len(visitedList)    # Proxy for memory usage
            
            # Record results
            results.append({
                "Direction": f"{direction}-Way",
                "Algorithm": name,
                "Trial": i,
                "Time (s)": duration,
                "Nodes Visited": nodes_visited,
                "Path Length": path_len
            })

# Compile results into a dataframe

df = pd.DataFrame(results)

# Filter out failed runs where there was no path found (in case the goal was unreachable)
valid_df = df[df["Path Length"] > 0]

# Group by direction and algorithm for side by side bar plots
summary = valid_df.groupby(["Direction", "Algorithm"]).agg({
    "Time (s)": "mean",
    "Nodes Visited": "mean",
    "Path Length": "mean"
}).reset_index()

# P rint summary table
print(summary)

# Visualisation

# Set style
plt.style.use('ggplot')
fig, axes = plt.subplots(1, 3, figsize=(18, 6))
fig.suptitle(f'Algorithm Performance Comparison ({trials} trials, {width}x{height} Grid)', fontsize=16)

# Helper to plot grouped bar charts
def plot_metric(plot, metric_name, title, ylabel):
    pivot_data = summary.pivot(index="Algorithm", columns="Direction", values=metric_name)
    pivot_data.plot(kind='bar', ax=plot, width=0.8)
    plot.set_title(title)
    plot.set_ylabel(ylabel)
    plot.set_xlabel("")
    plot.tick_params(axis='x', rotation=45)

# Time comparison
plot_metric(axes[0], "Time (s)", "Average Execution Time", "Seconds")

# Memory Usage (Nodes Visited)
plot_metric(axes[1], "Nodes Visited", "Memory Usage (Nodes Visited)", "Count")

# Path Length
plot_metric(axes[2], "Path Length", "Average Path Length", "Steps")

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()