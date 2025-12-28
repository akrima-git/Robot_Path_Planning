from collections import deque

class BFSGraph():
    def __init__(self, motion):
        self.motion = motion

    def traversal(self, grid, start, goal):
        motion = self.motion
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


class BFSTree():
    def __init__(self, motion, max_depth):
        self.motion = motion
        self.max_depth = max_depth

    def traversal(self, grid, start, goal):
        motion = self.motion
        max_depth = self.max_depth

        visitedList = []      # expansion order (can contain duplicates)
        visited = set()       # unique expanded states (for visualization only)

        queue = deque()
        queue.append((start, [start], 0))  # (state, path, depth)

        while queue:
            (y, x), path, depth = queue.popleft()

            # Mark expansion
            visitedList.append((y, x))
            visited.add((y, x))

            if (y, x) == goal:
                return path, visited, visitedList

            # Depth limit check
            if depth >= max_depth:
                continue

            for dy, dx in motion:
                ny, nx = y + dy, x + dx

                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] == 0:
                        queue.append(((ny, nx), path + [(ny, nx)], depth + 1))

        return None, visited, visitedList
