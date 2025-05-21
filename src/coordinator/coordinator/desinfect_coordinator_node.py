import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator
from coordinator.helper import FIPAMessage
from std_msgs.msg import String
from coordinator.FIPAPerformatives import FIPAPerformative


class Coordinator(AgnosticCoordinator):
    def __init__(self):
        super().__init__('desinfect Coordinator')
        #list of agents
        self.nurse = []
        self.spotrobot = []
        self.uvdrobot = []
        #list of agents that are currently occupied
        self.occ_nurse = []
        self.occ_spotrobot = []
        self.occ_uvdrobot = []