import heapq
import math

class AStarGraph:
    def __init__(self, motion):
        self.motion = motion

    def _heuristic(self, a, b, diagonal=False):     # Heuristic function (Euclidean or Manhattan)
        (y1, x1) = a
        (y2, x2) = b
        dy = abs(y1 - y2)
        dx = abs(x1 - x2)
        if diagonal:
            return math.hypot(dy, dx)
        else:
            return dy + dx

    def traversal(self, grid, start, goal):
        motion = self.motion
        diagonal = any(abs(dy) == 1 and abs(dx) == 1 for dy, dx in motion)  # Check if diagonal movement is allowed

        open_heap = []          # Min-heap priority queue for open set
        heapq.heappush(open_heap, (self._heuristic(start, goal, diagonal), 0, start))

        came_from = {}
        g_score = {start: 0}
        
        # Track expanded nodes to prevent re-expansion
        closed = set()
        visitedList = []

        if start == goal:                       # Immediate check for start equals goal
            return [start], closed, visitedList

        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            # If we have already processed this node, skip it
            if current in closed:
                continue
            
            closed.add(current)
            visitedList.append(current)

            if current == goal:                  # Reconstruct path
                path = [current]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                path.reverse()
                return path, closed, visitedList

            y, x = current      # Explore neighbours
            for dy, dx in motion:
                ny, nx = y + dy, x + dx
                                # Check bounds
                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] != 0:
                        continue

                    neighbour = (ny, nx)
                    step_cost = math.hypot(dy, dx)  # Cost to move to neighbour
                    tentative_g = g + step_cost

                    if neighbour in g_score and tentative_g >= g_score[neighbour]:
                        continue

                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbour, goal, diagonal)
                    heapq.heappush(open_heap, (f_score, tentative_g, neighbour))

        return None, closed, visitedList
    
class AStarTree:
    def __init__(self, motion):
        self.motion = motion

    def _heuristic(self, a, b, diagonal=False):
        (y1, x1) = a
        (y2, x2) = b
        dy = abs(y1 - y2)
        dx = abs(x1 - x2)
        if diagonal:
            return math.hypot(dy, dx)
        else:
            return dy + dx

    def traversal(self, grid, start, goal):
        motion = self.motion
        diagonal = any(abs(dy) == 1 and abs(dx) == 1 for dy, dx in motion)

        open_heap = []
        heapq.heappush(open_heap, (self._heuristic(start, goal, diagonal), 0, start))

        came_from = {}
        g_score = {start: 0}

        # Tree search
        expanded_unique = set() 
        visitedList = []

        if start == goal:
            return [start], expanded_unique, visitedList

        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            # As we don't have a closed set, we always process the node
            
            expanded_unique.add(current) 
            visitedList.append(current)

            if current == goal:
                path = [current]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                path.reverse()
                return path, expanded_unique, visitedList

            y, x = current
            for dy, dx in motion:
                ny, nx = y + dy, x + dx

                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] != 0:
                        continue

                    neighbour = (ny, nx)
                    step_cost = math.hypot(dy, dx)
                    tentative_g = g + step_cost

                    # Check if the new path is strictly worse to avoid infinite cycles
                    if neighbour in g_score and tentative_g >= g_score[neighbour]:
                        continue

                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbour, goal, diagonal)
                    heapq.heappush(open_heap, (f_score, tentative_g, neighbour))

        return None, expanded_unique, visitedList