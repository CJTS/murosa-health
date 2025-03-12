from murosa_plan_health.helper import FIPAMessage
from murosa_plan_health.FIPAPerformatives import FIPAPerformative
import rclpy
from rclpy.node import Node
from interfaces.srv import Message
from std_msgs.msg import String

class Coordinator(Node):
    def __init__(self):
        super().__init__('Coordinator')
        self.robots = []
        self.arms = []
        self.nurses = []

        # Coordinator server
        self._action_server = self.create_service(
            Message,
            'coordinator',
            self.execute_callback
        )
        self.jason_publisher = self.create_publisher(String, '/coordinator/jason/plan', 10)
        self.start_mission()

    def execute_callback(self, request, response):
        self.get_logger().info('Executing goal...')
        decoded_msg = FIPAMessage.decode(request.content)

        if decoded_msg.content == 'Register':
            if 'Arm' in decoded_msg.sender:
                id = str(len(self.arms) + 1)
                response.response = id
                self.arms.append(decoded_msg.sender + id)
            elif 'Robot' in decoded_msg.sender:
                id = str(len(self.robots) + 1)
                response.response = id
                self.robots.append(decoded_msg.sender + id)
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', 'Jason', 'Create|' + decoded_msg.sender + id).encode()
                self.jason_publisher.publish(msg)
            elif 'Nurse' in decoded_msg.sender:
                id = str(len(self.nurses) + 1)
                response.response = id
                self.nurses.append(decoded_msg.sender + id)

        self.get_logger().info(f'Received: Performative={decoded_msg.performative}, Sender={decoded_msg.sender}, Receiver={decoded_msg.receiver}, Content={decoded_msg.content}')
        return response

    def start_mission(self):
        msg = String()
        msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', 'Jason', 'Start').encode()
        self.jason_publisher.publish(msg)
        self.get_logger().info('Plans sent')
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
