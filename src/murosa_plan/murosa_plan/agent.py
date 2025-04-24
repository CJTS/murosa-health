import rclpy
from rclpy.node import Node
from interfaces.srv import Message, Action
from murosa_plan.FIPAPerformatives import FIPAPerformative
from murosa_plan.helper import FIPAMessage, action_string_to_tuple
from std_msgs.msg import String, Bool
from murosa_plan.ActionResults import ActionResult

class Agent(Node):
    def __init__(self, className):
        super().__init__(className)
        self.className = className.lower()
        self.actions = []
        self.plan = []
        self.wating_response = []
        self.wating = False

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

        # Subscriber para falar com o Jason (Ação)
        self.subscription_jason = self.create_subscription(
            String, '/jason/agent/action', self.listener_callback, 10
        )

        # Subscriber para falar com o Coordenador (Ação)
        self.subscription_coordinator = self.create_subscription(
            String, '/coordinator/agent/plan', self.listener_plan_callback, 10
        )

        # Publisher para falar o resultado da ação para o Jason
        self.publisher = self.create_publisher(String, '/agent/jason/result', 10)

        # Publisher para falar o resultado da ação para o coordenador
        self.publisher_coordinator = self.create_publisher(String, '/agent/coordinator/result', 10)

        # Subscriber para indicar fim da execução
        self.end_subscription = self.create_subscription(
            Bool, '/jason/shutdown_signal', self.shutdown_callback, 10
        )
        self.end_simulation_subscription = self.create_subscription(
            Bool, '/coordinator/shutdown_signal', self.end_simulation_callback, 10
        )

        # Publisher para falar com outros agentes
        self.agents_publisher = self.create_publisher(String, '/agent/agent/action', 10)

        # Subscriber para falar com outros agentes
        self.agents_subscription = self.create_subscription(
            String, '/agent/agent/action', self.respond_agent, 10
        )

        self.initialize()

    def initialize(self):
        # Send message to coordinator to be inserted in agent pools
        future = self.registration()
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.agentName = response.response
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
        if decoded_msg.content == 'Mission Completed':
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, self.agentName, 'Coordinator', 'Finished').encode()
            self.publisher_coordinator.publish(msg)
        else:
            self.add_action(decoded_msg)

    def listener_plan_callback(self, msg):
        # Receive messagem from jason
        self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        if not self.is_for_me(decoded_msg):
            self.get_logger().info('And it is not for me')
            return

        self.get_logger().info('And it is for me')
        ## Perform action
        message = decoded_msg.content.split('|')
        self.plan = list(map(action_string_to_tuple, message[1].split('/')))

    def is_for_me(self, msg):
        return msg.receiver == self.agentName

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, self.agentName, 'Jason', 'End|' + self.agentName).encode()
            self.publisher.publish(msg)
            raise SystemExit
        
    def end_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento do coordenador, finalizando...")
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, self.agentName, 'Jason', 'End|' + self.agentName).encode()
            self.publisher.publish(msg)
            raise SystemExit

    def ask_for_agent(self, agent, action):
        msg = String()
        msg.data = FIPAMessage(FIPAPerformative.QUERY.value, self.agentName, agent, 'Ready|' + action).encode()
        self.agents_publisher.publish(msg)
        self.wating_response.append((self.agentName, action))

    def acting_for_agent(self, agent, action):
        msg = String()
        msg.data = FIPAMessage(FIPAPerformative.QUERY.value, self.agentName, agent, 'Done|' + action).encode()
        self.agents_publisher.publish(msg)
        self.wating_response.append((self.agentName, action))

    def respond_agent(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        if not self.is_for_me(decoded_msg):
            self.get_logger().info('And it is not for me')
            return

        self.get_logger().info('And it is for me')
        if "Ready" == decoded_msg.content.split("|")[0]:
            if all(decoded_msg.content.split("|")[1] not in action for action in self.wating_response):
                self.get_logger().info('No there yet')
                self.wating_response.append((decoded_msg.sender, decoded_msg.content.split("|")[1]))
            else:
                self.get_logger().info('Ready to act')
                self.acting_for_agent(decoded_msg.sender, decoded_msg.content.split("|")[1])
        elif "Done" == decoded_msg.content.split("|")[0]:
            if len(self.actions) > 0:
                self.get_logger().info('Finished action')
                msg = String()
                action = self.actions.pop()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Success|' + ",".join(action)).encode()
                self.publisher.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
                self.wating = False
                self.acting_for_agent(decoded_msg.sender, decoded_msg.content.split("|")[1])
            else:
                self.get_logger().info('Already finished action')

    def add_action(self, msg):
        self.actions.append(msg.content.split(","))

    def act(self):
        if(len(self.plan) > 0) :
            action = self.plan.pop()
            result = self.choose_action(action)
            if result == ActionResult.WAITING:
                self.wating = True
                self.plan.append(action)
        elif(len(self.actions) > 0):
            self.get_logger().info('Acting')
            action = self.actions.pop()
            result = self.choose_action(action)
            if result == ActionResult.SUCCESS:
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Success|' + ",".join(action)).encode()
                self.publisher.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
            elif result == ActionResult.FAILURE:
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Failure|' + ",".join(action)).encode()
                self.publisher.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
            elif result == ActionResult.WAITING:
                self.wating = True
                self.actions.append(action)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            if not self.wating:
                self.act()
