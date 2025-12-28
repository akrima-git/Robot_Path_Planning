class DFSGraph():
    def __init__(self, motion):
        self.motion = motion

    def traversal(self, grid, start, goal):
        motion = self.motion

        visitedList = []
        visited = set()

        stack = []
        stack.append((start, [start]))

        while stack:
            (y, x), path = stack.pop()

            if (y, x) in visited:
                continue

            visited.add((y, x))
            visitedList.append((y, x))

            if (y, x) == goal:
                return path, visited, visitedList

            for dy, dx in motion:
                ny, nx = y + dy, x + dx

                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] == 0 and (ny, nx) not in visited:
                        stack.append(((ny, nx), path + [(ny, nx)]))

        return None, visited, visitedList

class DFSTree():
    def __init__(self, motion, max_depth):
        self.motion = motion
        self.max_depth = max_depth

    def traversal(self, grid, start, goal):
        motion = self.motion
        max_depth = self.max_depth

        visitedList = []
        visited = set()   # for visualization only

        stack = []
        stack.append((start, [start], 0))  # (state, path, depth)

        while stack:
            (y, x), path, depth = stack.pop()

            visitedList.append((y, x))
            visited.add((y, x))

            if (y, x) == goal:
                return path, visited, visitedList

            if depth >= max_depth:
                continue

            for dy, dx in motion:
                ny, nx = y + dy, x + dx

                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if grid[ny, nx] == 0:
                        stack.append(((ny, nx), path + [(ny, nx)], depth + 1))

        return None, visited, visitedList
