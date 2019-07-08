import os
import pybullet as p
from qibullet.robot_virtual import RobotVirtual
URDF_SPHERE = "data/robot_sphere.urdf"


class RobotSphere(RobotVirtual):
    """RobotSphere."""
    def __init__(self, pathImageTexture=None, rgba=None):
        RobotVirtual.__init__(self, URDF_SPHERE)
        self.path_image_texture = pathImageTexture
        self.rgba = rgba
        self.scale = 1
        self.texture_id = None

    def loadRobot(self, translation, quaternion, physicsClientId=0):
        """
        Overloads @loadRobot from the @RobotVirtual class.
        """
        self.physics_client = physicsClientId
        p.setAdditionalSearchPath(
            os.path.dirname(os.path.realpath(__file__)),
            physicsClientId=self.physics_client)
        self.robot_model = p.loadURDF(
            URDF_SPHERE,
            basePosition=translation,
            baseOrientation=quaternion,
            globalScaling=self.scale)
        self.setTexture(self.path_image_texture)
        self.setColor(self.rgba)
        p.changeDynamics(self.robot_model, -1, linearDamping=0.9,
                         physicsClientId=self.physics_client)

        # self.constraint = p.createConstraint(
        #                     self.robot_model, -1, -1, -1, p.JOINT_FIXED,
        #                     [0, 0, 0], [0, 0, 0], [1,1,1],
        #                     self.physics_client)

    def move(self, force):
        pos, orn = p.getBasePositionAndOrientation(
                                self.robot_model,
                                physicsClientId=self.physics_client)
        p.applyExternalForce(
                            self.robot_model, -1, force, pos,
                            flags=p.WORLD_FRAME,
                            physicsClientId=self.physics_client)

    def getDeformation(self):
        pass

    def getScale(self):
        return self.scale

    def getTextureId(self):
        return self.texture_id

    def getTexturePathName(self):
        return self.path_image_texture

    def setColor(self, rgba):
        if rgba is not None:
            p.changeVisualShape(self.robot_model, -1, rgbaColor=rgba,
                                physicsClientId=self.physics_client)
            self.rgba = rgba

    def setDeformation(self):
        pass

    def setScale(self, scale):
        self.scale = scale
        pos, orn = p.getBasePositionAndOrientation(
                                        self.robot_model,
                                        physicsClientId=self.physics_client)
        p.removeBody(self.robot_model, physicsClientId=self.physics_client)
        self.loadRobot(pos, orn, physicsClientId=self.physics_client)

    def setTexture(self, path_image):
        if path_image is not None and path_image != self.path_image_texture:
            self.texture_id = p.loadTexture(path_image,
                                        physicsClientId=self.physics_client)
            self.path_image_texture = path_image
        self.__setTexture()

    def __setTexture(self):
        if self.texture_id is not None:
            p.changeVisualShape(self.robot_model, -1,
                                textureUniqueId=self.texture_id,
                                physicsClientId=self.physics_client)
