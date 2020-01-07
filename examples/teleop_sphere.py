from modular_sphere import RobotSphere
from modular_sphere import SimulationManager
import pybullet as p
from time import sleep
from os.path import dirname, abspath

PARENT_PATH = dirname(dirname(abspath(__file__)))
DATA_PATH = PARENT_PATH + '/modular_sphere/data'
simulation_manager = SimulationManager()
physicsClient = simulation_manager.launchSimulation(gui=True)
sphere = simulation_manager.spawnRobotSphere(
                physicsClient,
                translation=[0, 0, 0.1], quaternion=[0, 0, 0, 1],
                 spawn_ground_plane=True)
forward = 0
turn = 0

forwardVec = 1.5
cameraDistance = 1
cameraYaw = 35
cameraPitch = -35
while (1):

  spherePos, orn = p.getBasePositionAndOrientation(sphere.robot_model)

  cameraTargetPosition = spherePos
  p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch,
                               cameraTargetPosition)
  camInfo = p.getDebugVisualizerCamera()
  camForward = camInfo[5]

  keys = p.getKeyboardEvents()
  for k, v in keys.items():
    if (k == p.B3G_F2 and (v & p.KEY_WAS_TRIGGERED)):
      sphere.setScale(2.0/2)
    if (k == p.B3G_F3 and (v & p.KEY_WAS_TRIGGERED)):
      sphere.setScale(2.0/3)
    if (k == p.B3G_F4 and (v & p.KEY_WAS_TRIGGERED)):
      sphere.setScale(2.0/4)
    if (k == p.B3G_F5 and (v & p.KEY_WAS_TRIGGERED)):
      sphere.setScale(2.0/5)

    if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      turn = -0.5
    if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
      turn = 0
    if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      turn = 0.5
    if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
      turn = 0

    if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      forward = 1
    if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
      forward = 0
    if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      forward = -1
    if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
      forward = 0

  force = [forward * camForward[0] * forwardVec,
           forward * camForward[1]* forwardVec, 0]
  cameraYaw = cameraYaw + turn
  sphere.move(force)
  sleep(1. / 240.)

simulation_manager.stopSimulation(physics_client=physicsClient)
