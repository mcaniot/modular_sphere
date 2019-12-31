import pybullet
from modular_sphere.robot_sphere import RobotSphere
from qibullet import simulation_manager
import threading

class SimulationManager(simulation_manager.SimulationManager):
    """
    Class allowing to handle the different parameters of a pybullet simulation
    """

    def __init__(self):
        """
        Constructor
        """
        simulation_manager.SimulationManager.__init__(self)

    def launchSimulation(self, gui=True):
        """
        Launches a simulation instance
        Parameters:
            gui - Boolean, if True the simulation is launched with a GUI, and
            with no GUI otherwise
        Returns:
            physics_client - The id of the simulation client created
        """
        if gui:
            physics_client = pybullet.connect(pybullet.GUI)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW,
                0,
                physicsClientId=physics_client)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                0,
                physicsClientId=physics_client)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
                0,
                physicsClientId=physics_client)
        else:
            physics_client = pybullet.connect(pybullet.DIRECT)
        threading.Thread(
            target=self._stepSimulation,
            args=[physics_client]).start()

        pybullet.setGravity(0, 0, -9.81, physicsClientId=physics_client)
        return physics_client

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
