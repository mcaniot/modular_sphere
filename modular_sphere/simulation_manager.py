import pybullet as p
from modular_sphere.robot_sphere import RobotSphere
from qibullet import simulation_manager

class SimulationManager(simulation_manager.SimulationManager):
    """
    Class allowing to handle the different parameters of a pybullet simulation
    """

    def __init__(self):
        """
        Constructor
        """
        simulation_manager.SimulationManager.__init__(self)

    def spawnRobotSphere(
                self,
                physics_client,
                translation=[0, 0, 0],
                quaternion=[0, 0, 0, 1],
                spawn_ground_plane=False):
            robot_sphere = RobotSphere()

            if spawn_ground_plane:
                self._spawnGroundPlane(physics_client)

            robot_sphere.loadRobot(
                translation,
                quaternion,
                physicsClientId=physics_client)

            return robot_sphere
