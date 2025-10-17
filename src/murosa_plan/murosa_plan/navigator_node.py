import heapq
import math
import rclpy
from rclpy.node import Node
from interfaces.srv import Action

import heapq
import math

class Navigator(Node):
    def __init__(self):
        super().__init__('Navigator')
        # === Nós principais (entradas e interseções)
        # formato: nome: (x, y)
        self.nodes = {
            "int1": (35, 80),
            "int2": (85, 80),
            "int3": (135, 80),
            "int4": (185, 80),
            "int5": (235, 80),

            "room1": (35, 50),
            "room2": (85, 50),
            "room3": (135, 50),
            "lab": (185, 50),
            "ds": (235, 50),
            "icu": (285, 80),

            "room4": (35, 110),
            "room5": (85, 110),
            "room6": (135, 110),
            "nr": (185, 110),
        }

        # === Conexões (grafo)
        self.graph = {
            "int1": ["int2", "room1", "room4"],
            "int2": ["int1", "int3", "room2", "room5"],
            "int3": ["int2", "int4", "room3", "room6"],
            "int4": ["int3", "int5", "lab", "nr"],
            "int5": ["int4", "icu", "ds"],
            "room1": ["int1"],
            "room2": ["int2"],
            "room3": ["int3"],
            "room4": ["int1"],
            "room5": ["int2"],
            "room6": ["int3"],
            "lab": ["int4"],
            "nr": ["int4"],
            "icu": ["int5"],
            "ds": ["int5"],
        }

        self.navigator_server = self.create_service(
            Action, 'navigator_server', self.receive_message
        )
        self.get_logger().info('Navigator server started')
    
    def receive_message(self, request, response):
        actionTuple = tuple(request.action.split(','))
        response.observation = 'success'
        
        return response

    def heuristic(self, a, b):
        (x1, y1) = self.nodes[a]
        (x2, y2) = self.nodes[b]
        return math.dist((x1, y1), (x2, y2))

    def astar(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor in self.graph[current]:
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []

    def compute_velocity(self, current_node, path, speed=1.0):
        if len(path) < 2:
            return (0.0, 0.0)

        next_node = path[1]
        x1, y1 = self.nodes[current_node]
        x2, y2 = self.nodes[next_node]
        dx, dy = x2 - x1, y2 - y1
        dist = math.sqrt(dx**2 + dy**2)
        if dist == 0:
            return (0.0, 0.0)
        return (dx/dist * speed, dy/dist * speed)

def main():
    rclpy.init()
    navigator = Navigator()
    try:
        rclpy.spin(navigator)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()