import rclpy

from rclpy.client import Client
from rclpy.node import Node

class MessageHelper():
    @staticmethod
    def send_client_message(client: Client, msg, node: Node):
        done = False
        while not done:
            future = client.call_async(msg)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            if not future.done():
                node.get_logger().info("Erro ao enviar mensagem, tentando novamente")
            else:
                done = True

        return future.result()


