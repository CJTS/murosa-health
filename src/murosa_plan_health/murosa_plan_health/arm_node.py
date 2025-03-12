import rclpy

from Agent import Agent

class Arm(Agent):
    def __init__(self, className):
        super().__init__(className)

def main():
    rclpy.init()
    robot = Agent('Robot')
    robot.get_logger().info('spin')
    try:
        rclpy.spin(robot)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    robot.get_logger().info('stop spin')
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()