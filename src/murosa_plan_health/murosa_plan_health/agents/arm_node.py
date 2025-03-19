import rclpy
from murosa_plan_health.agent import Agent

class Arm(Agent):
    def __init__(self, className):
        super().__init__(className)

def main():
    rclpy.init()
    arm = Arm('Arm')
    arm.get_logger().info('spin')
    try:
        rclpy.spin(arm)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    arm.get_logger().info('stop spin')
    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()