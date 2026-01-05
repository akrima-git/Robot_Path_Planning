from collections import deque


class BFSGraph:
    def __init__(self, motion):
        self.motion = motion

    def traversal(self, grid, start, goal):
        
        # Initialise set for visited nodes, and queue for the seen nodes to be visited
        visitedList = [] # For visualisation
        came_from = {}

        visited = set()
        visited.add(start)
        queue = deque()
        queue.append(start)
        
        # Immediate check for start equals goal
        if start == goal:
            return [start], visited, visitedList
        
        while queue:                    
            current = queue.popleft()
            visitedList.append(current)

            if current == goal: # If goal is found
                # Reconstruct path backwards from goal to start
                path = []
                while current in came_from: 
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path, visited, visitedList

            # Explore neighbours (4 or 8 depending on motion mode)
            y, x = current
            for dy, dx in self.motion:
                ny, nx = y + dy, x + dx
                # Check bounds and obstacles
                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] == 0 and (ny, nx) not in visited:
                        # Mark neighbour as visited and record its parent
                        visited.add((ny, nx))
                        came_from[(ny, nx)] = current
                        queue.append((ny, nx))


class BFSTree:
    def __init__(self, motion):
        self.motion = motion

    def traversal(self, grid, start, goal):

        # In tree search we store the path in the queue because we need to check strictly against the current branch's history, not the global visited set
        queue = deque()
        queue.append((start, [start])) 
        expanded_unique = set()
        
        # for visualisation
        visitedList = []

        # Store the best depth reached for each node to prevent re-expanding at greater depths
        best_depth = {start: 0}

        # Immediate check for start equals goal
        if start == goal:
            return [start], expanded_unique, visitedList

        while queue:
            (y, x), path = queue.popleft()

            current_depth = len(path) - 1
            
            expanded_unique.add((y, x))
            visitedList.append((y, x))

            # If the goal is found
            if (y, x) == goal:
                return path, expanded_unique, visitedList

            # Explore neighbours (4 or 8 depending on motion mode)
            for dy, dx in self.motion:
                ny, nx = y + dy, x + dx

                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] == 0:

                        # Calculate depth of the neighbour
                        next_depth = current_depth + 1

                        # Check if the neighbour is already in the current path
                        if (ny, nx) not in path:
                            # Only add neighbour if we have not reached it at a shallower depth before (path pruning)
                            if (ny, nx) not in best_depth or next_depth <= best_depth[(ny, nx)]:
                                best_depth[(ny, nx)] = next_depth
                                new_path = path + [(ny, nx)]
                                queue.append(((ny, nx), new_path))

        return None, expanded_unique, visitedList