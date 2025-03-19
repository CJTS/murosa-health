import rclpy
from rclpy.node import Node
from interfaces.srv import Message, Action
from murosa_plan_health.FIPAPerformatives import FIPAPerformative
from murosa_plan_health.helper import FIPAMessage
from std_msgs.msg import String, Bool

class Agent(Node):
    def __init__(self, className):
        super().__init__(className)
        self.className = className.lower()
        self.actions = []

        # Coordinator Client
        self.cli = self.create_client(Message, 'coordinator')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Environment Client
        self.environment_client = self.create_client(
            Action, 'environment_server'
        )
        while not self.environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('environment service not available, waiting again...')

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

        # Publisher para falar com outros agentes
        self.agents_publisher = self.create_publisher(String, '/agent/agent/action', 10)

        # Subscriber para falar com outros agentes
        self.agents_subscription = self.create_subscription(
            String, '/agent/agent/action', self.shutdown_callback, 10
        )

        self.initialize()

    def initialize(self):
        # Send message to coordinator to be inserted in agent pools
        future = self.registration()
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.agentName = str(self.className) + response.response
        self.get_logger().info('My name is %s' % (self.agentName))

    def registration(self):
        # Create FIPA message
        message = FIPAMessage(FIPAPerformative.REQUEST.value, self.className, 'Coordinator', 'Register').encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

    def listener_callback(self, msg):
        # Receive messagem from jason
        self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        if not self.is_for_me(decoded_msg):
            self.get_logger().info('And it is not for me')
            return

        self.get_logger().info('And it is for me')
        ## Perform action
        self.add_action(decoded_msg)

    def is_for_me(self, msg):
        return msg.receiver == self.agentName

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit

    def add_action(self, msg):
        self.actions.append(msg.content.split(","))

    def act(self):
        if(len(self.actions) > 0):
            self.get_logger().info('Acting')
            action = self.actions.pop()
            self.choose_action(action)
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Success|' + ",".join(action)).encode()
            self.publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            self.act()
