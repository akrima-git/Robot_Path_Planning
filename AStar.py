import heapq
import math

# PSEUDO CODE REFERENCE: https://www.geeksforgeeks.org/dsa/a-search-algorithm/
class AStarGraph:
    def __init__(self, motion):
        self.motion = motion

    def heuristic(self, a, b):                      # Heuristic function (Euclidean or Manhattan)
        diagonal = (1,1) in self.motion             # Check if diagonal movement is allowed
        dy = abs(a[0] - b[0])
        dx = abs(a[1] - b[1])
        if diagonal:
            return math.hypot(dy, dx)               # Euclidean distance for 8 movements
        else:
            return dy + dx                          # Manhattan distace for 4 movements

    def traversal(self, grid, start, goal):
  
        open_heap = []                              # Min-heap priority queue for open set
        heapq.heappush(open_heap, (self.heuristic(start, goal), 0, start))

        came_from = {}                              # For path reconstruction
        gScore = {start: 0}                         # Cost from start to node (dict for faster indexing)
        
        # Track closed set to avoid re-processing nodes
        closed = set()
        visitedList = []

        if start == goal:                           # Immediate check for start equals goal
            return [start], closed, visitedList

        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            # If we have already processed this node, skip it
            if current in closed:
                continue
            
            closed.add(current)
            visitedList.append(current)

            if current == goal:                     # Reconstruct path back to start (for visualisation) if goal reached
                path = [current]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                path.reverse()
                return path, closed, visitedList    # Exit if goal reached

            y, x = current                          # Explore neighbours
            for dy, dx in self.motion:
                ny, nx = y + dy, x + dx             # Check bounds
                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] != 0:           # Obstacle check
                        continue

                    neighbour = (ny, nx)
                    step_cost = math.hypot(dy, dx)  # Cost to move to neighbour
                    neighbourG = g + step_cost      # Total cost to move from start to neighbour

                    if neighbour in gScore and neighbourG >= gScore[neighbour]:
                        continue                    # Ignore if not a better path than what we already found

                    came_from[neighbour] = current  # Record best path to neighbour
                    gScore[neighbour] = neighbourG
                    f_score = neighbourG + self.heuristic(neighbour, goal) # Estimated total cost (from start to goal through neighbour)
                    heapq.heappush(open_heap, (f_score, neighbourG, neighbour))

        return None, closed, visitedList
    
class AStarTree:
    def __init__(self, motion):
        self.motion = motion

    def heuristic(self, a, b):                      # Heuristic function (Euclidean or Manhattan)
        diagonal = (1,1) in self.motion             # Check if diagonal movement is allowed
        dy = abs(a[0] - b[0])
        dx = abs(a[1] - b[1])
        if diagonal:
            return math.hypot(dy, dx)               # Euclidean distance for 8 movements
        else:
            return dy + dx                          # Manhattan distace for 4 movements

    def traversal(self, grid, start, goal):

        open_heap = []
        heapq.heappush(open_heap, (self.heuristic(start, goal), 0, start))

        cameFrom = {}
        gScore = {start: 0}

        # Tree search
        visitedUnique = set() 
        visitedList = []

        if start == goal:                           # Immediate check for start equals goal
            return [start], visitedUnique, visitedList
        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            # As we don't have a closed set, we always process the node
            
            visitedUnique.add(current) 
            visitedList.append(current)

            if current == goal:                # Reconstruct path for visualisation if goal reached
                path = [current]
                while path[-1] in cameFrom:
                    path.append(cameFrom[path[-1]])
                path.reverse()
                return path, visitedUnique, visitedList

            y, x = current                    # Explore neighbours
            for dy, dx in self.motion:
                ny, nx = y + dy, x + dx

                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:     # obstacle / boundary check
                    if grid[ny, nx] != 0:
                        continue

                    neighbour = (ny, nx)
                    step_cost = math.hypot(dy, dx)
                    neighbourG = g + step_cost

                    # Check if the new path is worse (to avoid infinite cycles)
                    if neighbour in gScore and neighbourG >= gScore[neighbour]:
                        continue

                    cameFrom[neighbour] = current   # Record best path to neighbour
                    gScore[neighbour] = neighbourG
                    fScore = neighbourG + self.heuristic(neighbour, goal)
                    heapq.heappush(open_heap, (fScore, neighbourG, neighbour))

        return None, visitedUnique, visitedList