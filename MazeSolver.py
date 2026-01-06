from pyamaze import maze, agent, COLOR, textLabel
from queue import PriorityQueue
from collections import deque


class MazeSolver:
    def __init__(self, m):
        self.m = m
        self.start = (m.rows, m.cols)
        self.goal = (1, 1)

    # Manhattan heuristic
    def heuristic(self, cell):
        x, y = cell
        return abs(x - 1) + abs(y - 1)

    # Get valid neighboring cells
    def get_neighbors(self, cell):
        neighbors = []
        x, y = cell

        for d in "ESNW":
            if self.m.maze_map[cell][d]:
                if d == 'E':
                    neighbors.append((x, y + 1))
                elif d == 'W':
                    neighbors.append((x, y - 1))
                elif d == 'N':
                    neighbors.append((x - 1, y))
                elif d == 'S':
                    neighbors.append((x + 1, y))
        return neighbors

    # ---------------- A* Algorithm ---------------- #
    def a_star(self):
        open_set = PriorityQueue()
        open_set.put((0, self.start))

        g = {cell: float('inf') for cell in self.m.grid}
        g[self.start] = 0

        parent = {}

        while not open_set.empty():
            _, current = open_set.get()
            if current == self.goal:
                break

            for neighbor in self.get_neighbors(current):
                temp_g = g[current] + 1
                f = temp_g + self.heuristic(neighbor)

                if f < g.get(neighbor, float('inf')):
                    g[neighbor] = temp_g
                    parent[neighbor] = current
                    open_set.put((f, neighbor))

        return self.reconstruct_path(parent)
    
    # ---------------- Path Reconstruction ---------------- #
    def reconstruct_path(self, parent):
        path = {}
        cell = self.goal
        while cell != self.start:
            path[parent[cell]] = cell
            cell = parent[cell]
        return path


# ---------------- MAIN PROGRAM ---------------- #

m = maze(10,10)
m.CreateMaze()
solver = MazeSolver(m)
a_star_path = solver.a_star()
a1 = agent(m, color=COLOR.blue, footprints=True, filled=True)
m.tracePath({a1: a_star_path})
textLabel(m, "A* Length", len(a_star_path) + 1)

m.run()