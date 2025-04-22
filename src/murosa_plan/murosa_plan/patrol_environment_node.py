from threading import Thread
import sys
import rclpy
from rclpy.node import Node
from interfaces.srv import Action
import json
import random
from std_msgs.msg import String, Bool

class Environment(Node):
    def __init__(self):
        super().__init__('Environment')
        self.declare_parameter('problem_rate', rclpy.Parameter.Type.INTEGER)
        self.start_server()
        self.state = {
            'loc': { 'r2d2': 'wp_control' },
            'connected': {
                'wp_control': ['wp1', 'wp2', 'wp3', 'wp4', 'wp5', 'wp6', 'wp7', 'wp8', 'wp9', 'wp10', 'wp11', 'wp12'],
                'wp1': ['wp_control'],
                'wp2': ['wp_control'],
                'wp3': ['wp_control'],
                'wp4': ['wp_control'],
                'wp5': ['wp_control'],
                'wp6': ['wp_control'],
                'wp7': ['wp_control'],
                'wp8': ['wp_control'],
                'wp9': ['wp_control'],
                'wp10': ['wp_control'],
                'wp11': ['wp_control'],
                'wp12': ['wp_control']
            },
            'patroled': { 'wp1': False, 'wp2': False, 'wp3': False, 'wp4': False, 'wp5': False, 'wp6': False, 'wp7': False, 'wp8': False, 'wp9': False, 'wp10': False, 'wp11': False, 'wp12': False }
        }

    def start_server(self):
        self.get_logger().info('Starting Environment server')
        self.environment_server = self.create_service(
            Action, 'environment_server', self.receive_message
        )
        self.end_subscription = self.create_subscription(
            Bool, '/shutdown_signal', self.shutdown_callback, 10
        )
        self.get_logger().info('Environment server started')

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit

    def receive_message(self, request, response):
        actionTuple = tuple(request.action.split(','))
        if actionTuple[0] == 'monitor':
            response.observation = json.dumps(self.state)
            return response
        self.get_logger().info('Responding')
        response.observation = 'success'
        return response

def main():
    rclpy.init()
    environment = Environment()
    environment.get_logger().info('spin')
    try:
        rclpy.spin(environment)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    environment.get_logger().info('stop spin')
    environment.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
