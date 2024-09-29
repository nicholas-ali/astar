"""
Nicholas Ali (nja3635)

The cost function is calculated through the 3D distance from one node to its successors.
The heuristic is calculated through the 3D distance a node to the finish node.
The above functions are used as they will give a good estimate of the total cost of moving to one node as
the purpose of the program is to find the shortest past from a start node, move through points, and reach the
end node.
"""

import math
import sys
from PIL import Image
from queue import PriorityQueue


class Node:
    def __init__(self, x: int, y: int, z: float, parent):
        self.x = x
        self.y = y
        self.z = z
        self.g = 0
        self.h = 0
        self.f = 0
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __hash__(self):
        return hash(str(self.x) + str(self.y) + str(self.x))

    def __str__(self):
        return "x: " + str(self.x) + " y: " + str(self.y) + " z: " + str(self.z)


def astar(start: Node, finish: Node, elevations, pixels):
    sqrt = math.sqrt
    frontier = PriorityQueue()
    frontier_map = dict()
    frontier.put(start)
    frontier_map[start] = start.g
    visited = set()
    while frontier:
        current = frontier.get()
        visited.add(current)
        if current == finish:
            return current
        neighbors = set()
        if current.x < 394:
            neighbors.add(Node(current.x + 1, current.y, float(elevations[current.y][current.x + 1]), current))
        if current.y < 499:
            neighbors.add(Node(current.x, current.y + 1, float(elevations[current.y + 1][current.x]), current))
        if current.x > 0:
            neighbors.add(Node(current.x - 1, current.y, float(elevations[current.y][current.x - 1]), current))
        if current.y > 0:
            neighbors.add(Node(current.x, current.y - 1, float(elevations[current.y - 1][current.x]), current))
        for neighbor in neighbors:
            if neighbor not in visited:
                terrain = pixels.getpixel((neighbor.x, neighbor.y))
                penalty = 0
                match terrain:
                    case (248, 148, 18):
                        penalty = 30
                    case (255, 192, 0):
                        penalty = 100
                    case (255, 255, 255):
                        penalty = 80
                    case (2, 208, 60):
                        penalty = 80
                    case (2, 136, 40):
                        penalty = 100
                    case (5, 73, 24):
                        penalty = 400
                    case (0, 0, 255):
                        penalty = 400
                    case (71, 51, 3):
                        penalty = 0
                    case (0, 0, 0):
                        penalty = 20
                    case (205, 0, 101):
                        penalty = 99999
                    case _:
                        penalty = 0
                neighbor.g = current.g + sqrt(
                    (current.x * 10.29 - neighbor.x * 10.29) ** 2 + (current.y * 7.55 - neighbor.y * 7.55) ** 2 + (
                                current.z - neighbor.z) ** 2)
                neighbor.h = sqrt(
                    (finish.x * 10.29 - neighbor.x * 10.29) ** 2 + (finish.y * 7.55 - neighbor.y * 7.55) ** 2 + (
                                finish.z - neighbor.z) ** 2)
                neighbor.f = neighbor.g + neighbor.h + penalty
                if neighbor in frontier_map:
                    if neighbor.g > frontier_map[neighbor]:
                        continue

                frontier.put(neighbor)
                frontier_map[neighbor] = neighbor.g
    return None


def main():
    terrain_image = sys.argv[1]
    elevation_file = sys.argv[2]
    path_file = sys.argv[3]
    output_image_filename = sys.argv[4]
    terrain = Image.open(terrain_image)
    pixels = terrain.load()
    rgb_terrain = terrain.convert('RGB')
    elevations = []
    with open(elevation_file) as file:
        for line in file:
            elevations.append(line.split()[:395])
    path = list()
    with open(path_file) as file:
        for line in file:
            path.append(line.split())
    total = 0
    for i in range(len(path) - 1):
        start = Node(int(path[i][0]), int(path[i][1]), float(elevations[int(path[i][1])][int(path[i][0])]), None)
        finish = Node(int(path[i + 1][0]), int(path[i + 1][1]),
                      float(elevations[int(path[i + 1][1])][int(path[i + 1][0])]), None)
        sol = astar(start, finish, elevations, rgb_terrain)
        distances = list()
        while sol:
            distances.append(sol.g)
            pixels[sol.x, sol.y] = (118, 63, 231, 255)
            sol = sol.parent
        total += max(distances)
    print("total: " + str(total))
    terrain.save(output_image_filename)
    terrain.show()


if __name__ == '__main__':
    main()
