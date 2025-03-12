import random
from threading import Thread

from murosa_plan_health.helper import FIPAMessage

import rclpy
from rclpy.node import Node

from interfaces.srv import Message

from std_msgs.msg import String, Bool

class Coordinator(Node):
    def __init__(self):
        super().__init__('Coordinator')

        # Coordinator server
        self._action_server = self.create_service(
            Message,
            'coordinator',
            self.execute_callback
        )

    def execute_callback(self, request, response):
        self.get_logger().info('Executing goal...')
        decoded_msg = FIPAMessage.decode(request.content)
        self.get_logger().info(f'Received: Performative={decoded_msg.performative}, Sender={decoded_msg.sender}, Receiver={decoded_msg.receiver}, Content={decoded_msg.content}')
        response.response = 'ok'
        return response

    def start_sim(self):
        return

def main():
    rclpy.init()
    coordinator = Coordinator()
    coordinator.get_logger().info('spin')
    try:
        rclpy.spin(coordinator)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    coordinator.get_logger().info('stop spin')
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
