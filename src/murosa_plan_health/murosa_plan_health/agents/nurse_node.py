import rclpy
import time
from random import randrange
from murosa_plan_health.agent import Agent
from murosa_plan_health.helper import FIPAMessage
from interfaces.srv import Message
from murosa_plan_health.FIPAPerformatives import FIPAPerformative

class Nurse(Agent):
    def __init__(self, className):
        super().__init__(className)
        self.has_sample = False
        self.run()

    def run(self):
        while(True):
            if not self.has_sample:
                self.has_sample = True
                seconds = randrange(10)
                time.sleep(seconds)
                self.send_has_sample_message()

    def send_has_sample_message(self):
        message = FIPAMessage(FIPAPerformative.INFORM.value, self.className, 'Coordinator', 'HasSample').encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

def main():
    rclpy.init()
    nurse = Nurse('Nurse')
    nurse.get_logger().info('spin')
    try:
        rclpy.spin(nurse)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    nurse.get_logger().info('stop spin')
    nurse.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()