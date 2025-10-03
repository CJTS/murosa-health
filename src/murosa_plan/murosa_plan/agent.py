import rclpy
import time
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
        self.declare_parameter('bdi', rclpy.Parameter.Type.BOOL)
        self.should_use_bdi = self.get_parameter('bdi').get_parameter_value().bool_value
        self.with_plan = False

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
        if self.should_use_bdi:
            self.subscription_jason = self.create_subscription(
                String, '/jason/agent/action', self.listener_callback, 10
            )

        # Subscriber para falar com o Coordenador (Ação)
        if not self.should_use_bdi:
            self.subscription_coordinator = self.create_subscription(
                String, '/coordinator/agent/plan', self.listener_plan_callback, 10
            )

        # Publisher para falar o resultado da ação para o Jason
        if self.should_use_bdi:
            self.publisher = self.create_publisher(String, '/agent/jason/result', 10)

        # Publisher para falar o resultado da ação para o coordenador
        self.publisher_coordinator = self.create_publisher(String, '/agent/coordinator/result', 10)

        # Subscriber para indicar fim da execução
        if self.should_use_bdi:
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
        self.get_logger().info('response.response %s' % (response.response))
        self.get_logger().info('My name is %s' % (self.agentName))

    def registration(self):
        # Create FIPA message
        message = FIPAMessage(FIPAPerformative.REQUEST.value, self.className, 'Coordinator', 'Register').encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

    def listener_callback(self, msg):
        # Receive messagem from jason
        # self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        if not self.is_for_me(decoded_msg):
            # self.get_logger().info('And it is not for me')
            return

        # self.get_logger().info('And it is for me')
        ## Perform action
        if decoded_msg.content == 'Mission Completed':
            self.end_local_mission()
        else:
            self.add_action(decoded_msg)

    def listener_plan_callback(self, msg):
        # Receive messagem from jason
        # self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        if not self.is_for_me(decoded_msg):
            # self.get_logger().info('And it is not for me')
            return

        # self.get_logger().info('And it is for me')
        ## Perform action
        message = decoded_msg.content.split('|')
        self.plan = list(map(action_string_to_tuple, message[1].split('/')))
        self.with_plan = True

    #colocado para disinfect
    def listener_reset_callback(self, msg):
        # Receive messagem from jason
        # self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        if not self.is_for_me(decoded_msg):
            # self.get_logger().info('And it is not for me')
            return

        # self.get_logger().info('And it is for me')
        ## Perform action
        self.actions = []
        self.plan = []
        self.wating_response = []
        self.wating = False
    
    def end_local_mission(self):
        self.get_logger().info('Mission Completed, resetting local state')
        self.actions = []
        self.plan = []
        self.wating_response = []
        self.wating = False
        msg = String()
        msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, self.agentName, 'Coordinator', 'Finished').encode()
        self.publisher_coordinator.publish(msg)


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
        # self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        if not self.is_for_me(decoded_msg):
            # self.get_logger().info('And it is not for me')
            return

        # self.get_logger().info('And it is for me')
        if "Ready" == decoded_msg.content.split("|")[0]:
            if all(decoded_msg.content.split("|")[1] not in action for action in self.wating_response):
                self.get_logger().info('No there yet ' + decoded_msg.content.split("|")[1])
                self.wating_response.append((decoded_msg.sender, decoded_msg.content.split("|")[1]))
            else:
                self.get_logger().info('Ready to act ' + decoded_msg.content.split("|")[1])
                self.acting_for_agent(decoded_msg.sender, decoded_msg.content.split("|")[1])
        elif "Done" == decoded_msg.content.split("|")[0]:
            # Check if the action is in the actions or plans
            if len(self.actions) > 0 or any(decoded_msg.content.split("|")[1] in action for action in self.plan):
                self.get_logger().info('Finished action ' + decoded_msg.content.split("|")[1])
                if self.should_use_bdi:
                    msg = String()
                    action = self.actions.pop()
                    msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Success|' + ",".join(action)).encode()
                    self.publisher.publish(msg)
                else:
                    action = self.plan.pop(0)

                # self.get_logger().info('Publishing: "%s"' % msg.data)
                self.wating = False
                self.acting_for_agent(decoded_msg.sender, decoded_msg.content.split("|")[1])
            else:
                self.get_logger().info('Already finished action ' + decoded_msg.content.split("|")[1])

    def add_action(self, msg):
        self.get_logger().info('Adding action: ' + msg.content)
        self.actions.append(msg.content.split(","))

    def act(self):
        time.sleep(0.2)  # To avoid overloading the CPU
        if(len(self.plan) > 0):
            action = self.plan.pop(0)
            result = self.choose_action(action)
            if result == ActionResult.WAITING:
                self.wating = True
                self.plan.insert(0, action)
        elif(len(self.actions) > 0 and self.should_use_bdi):
            self.get_logger().info('Acting')
            action = self.actions.pop()
            result = self.choose_action(action)
            if result == ActionResult.SUCCESS:
                # self.get_logger().info("ActionResult.SUCCESS")
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Success|' + ",".join(action)).encode()
                self.publisher.publish(msg)
                # self.get_logger().info('Publishing: "%s"' % msg.data)
            elif result == ActionResult.FAILURE:
                # self.get_logger().info("ActionResult.FAILURE")
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Failure|' + ",".join(action)).encode()
                self.publisher.publish(msg)
                # self.get_logger().info('Publishing: "%s"' % msg.data)
            elif result == ActionResult.BATTERY_FAILURE:
                # self.get_logger().info("ActionResult.BATTERY_FAILURE")
                """Send battery failure to Jason"""
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'BatteryFailure|'+ ",".join(action)).encode()
                self.publisher.publish(msg)
                # self.get_logger().info('Publishing: "%s"' % msg.data)
            elif result == ActionResult.WAITING:
                # self.get_logger().info("ActionResult.WAITING")
                self.wating = True
                self.actions.append(action)
        
        if len(self.plan) == 0 and self.with_plan:
            self.end_local_mission()
            self.with_plan = False

    def notifyError(self, error):
        message = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Coordinator', 'ERROR|' + error).encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        future = self.cli.call_async(ros_msg)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info('%s' % (response.response))

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            if not self.wating:
                self.act()
