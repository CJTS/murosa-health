import uuid
import rclpy
from rclpy.node import Node
from interfaces.srv import Message
from murosa_plan_health.FIPAPerformatives import FIPAPerformative
from murosa_plan_health.helper import FIPAMessage
from std_msgs.msg import String, Bool

class Agent(Node):
    def __init__(self, className):
        super().__init__(className)
        self.className = className
        self.agentId = uuid.uuid4()
        self.agentName = str(className) + '-' + str(self.agentId)

        # Coordinator Server
        self.cli = self.create_client(Message, 'coordinator')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # Subscriber para falar com o Jason (Ação move)

        self.subscription = self.create_subscription(
            String, '/jason/agent/action', self.listener_callback, 10
        )

        # Publisher para falar o resultado da ação
        self.publisher = self.create_publisher(String, '/agent/jason/result', 10)

        # Subscriber para indicar fim da execução
        self.end_subscription = self.create_subscription(
            Bool, '/jason/agent/shutdown_signal', self.shutdown_callback, 10
        )

        self.start()

    def start(self):
        # Send message to coordinator to be inserted in agent pools
        future = self.registration()
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info('Response %s' % (response.response))

    def registration(self):
        # Create FIPA message
        message = FIPAMessage(FIPAPerformative.REQUEST.value, self.agentName, 'Coordinator', 'Register').encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

    def listener_callback(self, msg):
        # Receive messagem from jason
        self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        response = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Coordinator').encode()
        self.publisher.publish(response)
        self.get_logger().info('Publishing: "%s"' % response.data)

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit
