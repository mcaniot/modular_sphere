#!/usr/bin/env python
# coding: utf-8
from modular_sphere import SimulationManager
import os
import pybullet as p
import time
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
        except Exception as error:
            print("Normal error", error)
        self.assertTrue(
            RobotSphereClassTest.robot.getTextureId() is None)
        self.assertTrue(
            RobotSphereClassTest.robot.getTexturePathName() !=\
            texture_path_name)
        texture_path_name = "data/texture.png"
        RobotSphereClassTest.robot.setTexture(texture_path_name)
        time.sleep(1)
        shape_data =\
            p.getVisualShapeData(RobotSphereClassTest.robot.getRobotModel(),
                                 flags=p.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS,
                                 physicsClientId=RobotSphereClassTest.client)
        # Texture id is not updating in pybullet code, to check
        # self.assertTrue(shape_data[0][-1] != -1)
        self.assertTrue(
            RobotSphereClassTest.robot.getTexturePathName() ==\
            texture_path_name)

    def test_move(self):
        robot_pose_init = RobotSphereClassTest.robot.getPosition()
        RobotSphereClassTest.robot.move([1, 0, 0])
        time.sleep(0.5)
        robot_pose_actual = RobotSphereClassTest.robot.getPosition()
        self.assertTrue(robot_pose_actual[0] != robot_pose_init[0])
        self.assertTrue(robot_pose_actual[1] == robot_pose_init[1])
        self.assertTrue(robot_pose_actual[2] == robot_pose_init[2])

    def test_deformation(self):
        # not implemented yet, for now the functions are empty
        RobotSphereClassTest.robot.setDeformation()
        RobotSphereClassTest.robot.getDeformation()

    def test_scale_dimension(self):
        robot_pose_init = RobotSphereClassTest.robot.getPosition()
        init_dimension = RobotSphereClassTest.robot.getDimension()
        RobotSphereClassTest.robot.setScale(2)
        self.assertTrue(RobotSphereClassTest.robot.getScale() == 2)
        robot_pose_actual = RobotSphereClassTest.robot.getPosition()
        self.assertTrue(robot_pose_actual == robot_pose_init)
        shape_data =\
            p.getVisualShapeData(RobotSphereClassTest.robot.getRobotModel(),
                                 physicsClientId=RobotSphereClassTest.client)
        actual_dimensions = shape_data[0][3]
        self.assertTrue(
            init_dimension*RobotSphereClassTest.robot.getScale() ==\
            actual_dimensions[0])
        

if __name__ == "__main__":
    unittest.main()
