from threading import Thread
import rclpy
from rclpy.node import Node
from interfaces.srv import HasSample, SendPlan, Action
from murosa_plan_health.helper import action_string_to_tuple


class Nurse(Node):

    def __init__(self):
        super().__init__('Nurse')
        self.local_plan = []
        self.current_action = ()
        self.new_plan = True

    def start_server(self):
        self.get_logger().info('Starting Nurse server')
        self.nurse_server = self.create_service(
            SendPlan, 'nurse_server', self.receive_message
        )
        self.get_logger().info('Nurse server started')

    def receive_message(self, request, response):
        self.get_logger().info('Receiving message.')
        self.get_logger().info("Nurse's actions:")
        self.new_plan = not self.new_plan
        self.local_plan = list(map(action_string_to_tuple, request.plan))
        for action in self.local_plan:
            self.get_logger().info(action[0])

        response.ok = 'Okay!'
        return response

    def start_client(self):
        self.get_logger().info('Starting Nurse clients')
        self.coordinator_client = self.create_client(
            HasSample, 'coordinator_server'
        )
        while not self.coordinator_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('coordinator service not available, waiting again...')

        self.environment_client = self.create_client(
            Action, 'environment_server'
        )
        while not self.environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('environment service not available, waiting again...')

        self.robot_communication_sync_client = self.create_client(
            Action, 'robot_communication_sync_server'
        )
        while not self.robot_communication_sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('robot service not available, waiting again...')
        self.get_logger().info('Nurse clients started')

    def send_has_sample_request(self, nurseid, roomid):
        self.has_sample_request = HasSample.Request()
        self.has_sample_request.nurseid = nurseid
        self.has_sample_request.roomid = roomid
        return self.coordinator_client.call(self.has_sample_request)

    def has_sample(self):
        self.get_logger().info('Sent request')
        response = self.send_has_sample_request('nurse1', 'room3')
        self.get_logger().info('Request response:%s' % (response.ok))

    def run(self):
        while rclpy.ok():
            # local plan
            if len(self.local_plan) > 0:
                self.current_action = self.local_plan.pop(0)
                self.get_logger().info("Action: %s" % (self.current_action[0]))
                ok = self.choose_action(self.current_action)
                self.get_logger().info("Action result: %s" % (ok))
                if not ok and not self.new_plan:
                    self.local_plan.insert(0, self.current_action)
                self.new_plan = False
    # ACTIONS

    def choose_action(self, actionTuple):
        if actionTuple[0] == 'a_authenticate_nurse':
            self.get_logger().info('Doing a_authenticate_nurse')
            response = self.a_authenticate_nurse(
                actionTuple[1], actionTuple[2]
            )
            self.get_logger().info(response.observation)
        elif actionTuple[0] == 'a_open_door':
            self.get_logger().info('Doing a_open_door')
            response = self.a_open_door(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)
        elif actionTuple[0] == 'a_deposit':
            self.get_logger().info('Doing a_deposit')
            response = self.a_deposit(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)

        if not response.observation == 'success':
            return False
        return True

    def a_authenticate_nurse(self, robot, nurse):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_authenticate_nurse', robot, nurse)
        )
        return self.robot_communication_sync_client.call(
            self.action_request
        )

    def a_open_door(self, nurse, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_open_door', nurse, room)
        )
        return self.environment_client.call(self.action_request)

    def a_deposit(self, nurse, robot):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_deposit', nurse, robot)
        )
        return self.robot_communication_sync_client.call(
            self.action_request
        )


def main():
    rclpy.init()
    nurse = Nurse()
    nurse.start_server()
    nurse.start_client()
    spin_thread = Thread(target=rclpy.spin, args=(nurse,))
    spin_thread.start()
    nurse.has_sample()
    nurse.run()
    nurse.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
