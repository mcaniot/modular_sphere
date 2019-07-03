import pybullet as p

URDF_SPHERE = "sphere_smooth.urdf"
class RobotSphere(object):
    """docstring for RobotSphere."""
    def __init__(self):
        self.scale = 1
        self.robot_model = p.loadURDF(
            URDF_SPHERE,
            basePosition=[0,0,1],
            baseOrientation=[0,0,0,1],
            globalScaling=self.scale)
        p.changeDynamics(self.robot_model, -1, linearDamping=0.9)
        texUid = p.loadTexture("tex.png")
        p.changeVisualShape(self.robot_model, -1, textureUniqueId=texUid)

    def setScale(self, scale):
        self.scale = scale
        pos, orn = p.getBasePositionAndOrientation(self.robot_model)
        p.removeBody(self.robot_model)
        self.robot_model = p.loadURDF(
            URDF_SPHERE,
            basePosition=pos,
            baseOrientation=orn,
            globalScaling=self.scale)
        texUid = p.loadTexture("tex.png")
        p.changeVisualShape(self.robot_model, -1, textureUniqueId=texUid)

    def getScale(self):
        return self.scale

    def move(self, force):
        pos, orn = p.getBasePositionAndOrientation(self.robot_model)
        p.applyExternalForce(
                            self.robot_model, -1, force, pos,
                            flags=p.WORLD_FRAME)

    def getDeformation(self):
        pass

    def changeColor(self, rgba):
        p.changeVisualShape(self.robot_model, -1, rgbaColor=rgba)
