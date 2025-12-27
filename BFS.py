from collections import deque

class BFS1():
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
