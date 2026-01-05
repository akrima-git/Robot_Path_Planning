class DFSGraph:
    def __init__(self, motion):
        self.motion = motion

    def traversal(self, grid, start, goal):

        visited = set()
        visitedList = [] # For visualisation

        stack = []
        stack.append(start)
        
        # allows for path reconstruction
        came_from = {}

        while stack:
            current = stack.pop()

            # Ignore visited nodes
            if current in visited:
                continue

            visited.add(current)
            visitedList.append(current)

            if current == goal:
                # Reconstruct path backwards
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path, visited, visitedList

            y, x = current
            
            # Explore neighbours (4 or 8 depending on motion mode)
            for dy, dx in self.motion:
                ny, nx = y + dy, x + dx
                # Check bounds and obstacles
                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] == 0 and (ny, nx) not in visited:
                        # Add neighbour to stack and record its parent
                        came_from[(ny, nx)] = current
                        stack.append((ny, nx))

        return None, visited, visitedList




class DFSTree:
    def __init__(self, motion, max_depth = 1000): 
        self.motion = motion
        self.max_depth = max_depth

    def traversal(self, grid, start, goal):
        motion = self.motion
        max_depth = self.max_depth

        visitedList = []      
        visited = set()       # unique nodes (visualisation only)

        stack = []
        stack.append((start, [start]))

        while stack:
            (y, x), path = stack.pop()
            
            # Calculate current depth for depth limiting
            depth = len(path)

            visitedList.append((y, x))
            visited.add((y, x))

            if (y, x) == goal:
                return path, visited, visitedList

            # Stop if we hit depth limit
            if depth >= max_depth:
                continue
            
            # explore neighbours (4 or 8 depending on motion mode)
            for dy, dx in motion:
                ny, nx = y + dy, x + dx

                # Check bounds and obstacles
                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] == 0:
                        # Make sure node isnt in current path
                        if (ny, nx) not in path:
                            new_path = path + [(ny, nx)]
                            stack.append(((ny, nx), new_path))

        return None, visited, visitedList
    

