#!/usr/bin/env python
# coding: utf-8
from modular_sphere import SimulationManager
import os
import pybullet as p
import unittest

class RobotSphereClassTest(unittest.TestCase):
    """
    Unittests for the control of Pepper virtual's base
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        RobotSphereClassTest.simulation = SimulationManager()
        RobotSphereClassTest.client =\
            RobotSphereClassTest.simulation.launchSimulation(
            gui=False)

        RobotSphereClassTest.robot =\
            RobotSphereClassTest.simulation.spawnRobotSphere(
            RobotSphereClassTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        RobotSphereClassTest.simulation.stopSimulation(
            RobotSphereClassTest.client)

    def test_color(self):
        black_color = (0, 0, 0, 1)
        white_color = (1, 1, 1, 1)
        RobotSphereClassTest.robot.setColor(black_color)
        shape_data =\
            p.getVisualShapeData(RobotSphereClassTest.robot.getRobotModel(),
                                 physicsClientId=RobotSphereClassTest.client)
        self.assertTrue(black_color == shape_data[0][-1])
        RobotSphereClassTest.robot.setColor(white_color)
        shape_data =\
            p.getVisualShapeData(RobotSphereClassTest.robot.getRobotModel(),
                                 physicsClientId=RobotSphereClassTest.client)
        self.assertTrue(white_color == shape_data[0][-1])

    def test_texture(self):
        p.setAdditionalSearchPath(
            os.path.dirname(os.path.realpath(__file__)),
            physicsClientId=RobotSphereClassTest.client)
        texture_path_name = "This is not a texture"
        try:
            RobotSphereClassTest.robot.setTexture(texture_path_name)
        except Exception as e:
            pass
        self.assertTrue(
            RobotSphereClassTest.robot.getTextureId() is None)
        self.assertTrue(
            RobotSphereClassTest.robot.getTexturePathName() !=\
            texture_path_name)
        texture_path_name = "data/texture.png"
        RobotSphereClassTest.robot.setTexture(texture_path_name)
        shape_data =\
            p.getVisualShapeData(RobotSphereClassTest.robot.getRobotModel(),
                                 flags=p.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS,
                                 physicsClientId=RobotSphereClassTest.client)
        self.assertTrue(
            RobotSphereClassTest.robot.getTexturePathName() ==\
            texture_path_name)


if __name__ == "__main__":
    unittest.main()
