import heapq
import math

# version of Uniform Cost Search adapted from A* implementation and https://www.geeksforgeeks.org/artificial-intelligence/uniform-cost-search-ucs-in-ai/
class UCSGraph:
    def __init__(self, motion):
        self.motion = motion

    def traversal(self, grid, start, goal):
        # No heuristic function needed for UCS

        # Priority queue (gScore, (y, x))
        # Unlike A*, we only care about g (cost from start)
        open_heap = []
        heapq.heappush(open_heap, (0, start))

        came_from = {}
        gScore = {start: 0}

        # Track expanded nodes to prevent re-processing of nodes
        closed = set()
        visitedList = []                    # Order of visited nodes for visualisation

        if start == goal:                   # Immediate check for start equals goal
            return [start], closed, visitedList

        while open_heap:
            # Pop the node with the lowest cumulative cost (g)
            g, current = heapq.heappop(open_heap)

            # If we have already finalised this node, skip it
            if current in closed:
                continue

            closed.add(current)
            visitedList.append(current)

            if current == goal:                # Reconstruct path back to start (for visualisation) if goal reached
                path = [current]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                path.reverse()
                return path, closed, visitedList

            y, x = current
            for dy, dx in self.motion:        # Explore neighbours for each direction (4 or 8 depending on motion mode)
                ny, nx = y + dy, x + dx

                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] != 0:     # Obstacle check
                        continue

                    neighbour = (ny, nx)
                    
                    # Calculate cost to move to neighbour
                    step_cost = math.hypot(dy, dx)
                    neighbourG = g + step_cost

                    # If this is a new path or a shorter path to neighbour
                    if neighbour not in gScore or neighbourG < gScore[neighbour]:
                        came_from[neighbour] = current
                        gScore[neighbour] = neighbourG
                        heapq.heappush(open_heap, (neighbourG, neighbour))

        return None, closed, visitedList
    


class UCSTree:
    def __init__(self, motion):
        self.motion = motion

    def traversal(self, grid, start, goal):
        
        # Priority queue: (g_score, (y, x))
        open_heap = []
        heapq.heappush(open_heap, (0, start))

        came_from = {}
        gScore = {start: 0}

        # No closed set for tree search
        expanded_unique = set()
        visitedList = []

        if start == goal:                   # Immediate check for start equals goal
            return [start], expanded_unique, visitedList

        while open_heap:
            g, current = heapq.heappop(open_heap)
            
            expanded_unique.add(current)
            visitedList.append(current)

            if current == goal:                         # Reconstruct path back to start (for visualisation) if goal reached
                path = [current]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                path.reverse()
                return path, expanded_unique, visitedList

            y, x = current
            for dy, dx in self.motion:
                ny, nx = y + dy, x + dx

                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:  # Check bounds
                    if grid[ny, nx] != 0:               # Obstacle check
                        continue

                    neighbour = (ny, nx)                
                    step_cost = math.hypot(dy, dx)
                    neighbourG = g + step_cost

                    # If we found a cheaper path to the neighbour ealier, don't add it (avoids cyclic loops)
                    if neighbour in gScore and neighbourG >= gScore[neighbour]:
                        continue

                    came_from[neighbour] = current      # Record best path to neighbour
                    gScore[neighbour] = neighbourG
                    heapq.heappush(open_heap, (neighbourG, neighbour))

        return None, expanded_unique, visitedList