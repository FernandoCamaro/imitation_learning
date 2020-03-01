import pybullet as p
import pybullet_data
import os.path as path

class kukaEnv():
    def __init__(self, mode="GUI"):
        assert mode in ["GUI","SHARED_MEMORY","DIRECT"], "mode not supported"
        if mode == "GUI":
            cid = p.connect(p.GUI)
        elif mode == "SHARED_MEMORY":
            cid = p.connect(p.SHARED_MEMORY)
        elif mode == "DIRECT":
            cid = p.connect(p.DIRECT)
        assert cid >= 0, "Pybullet couldn't connect."
    
        p.resetSimulation()
        # disable rendering during loading makes it much faster
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setInternalSimFlags(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setAdditionalSearchPath(path.abspath("cad_models"))

        # plane
        plane = p.loadURDF("plane.urdf", 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)

        # kuka
        kuka = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000, -0.000000, 0.600000, 0.000000, 0.000000, 0.000000, 1.000000)
        kuka_ee = 6

        # gripper
        kuka_gripper = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
        p.resetBasePositionAndOrientation(kuka_gripper, [0.923103, -0.000000, 1.250036], [-0.000000, 0.964531, -0.000002, -0.263970])

        # table
        table = p.loadURDF("table/table.urdf", 1.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.707107,0.707107)

        # cube
        self._RESET_CUBE_LOC = [0.9499039814386195, -0.09998764359191646, 0.6499880447702621]
        self._RESET_CUBE_ORI = [0.000000, 0.000000, 0.707107, 0.707107]
        cube = p.loadURDF("cube_small.urdf", self._RESET_CUBE_LOC, self._RESET_CUBE_ORI)

        # bowl
        self._RESET_BOWL_LOC = [0.8325537415161121, 0.28105872079567157, 0.4860074457860908]
        self._RESET_BOWL_ORI =  [0.6998614318582163, 0.10134281986907948, 0.10145490068854048, 0.6997360303604704]
        #bowl_path = "C:/Users/ascent/Documents/imitation_learning/cad_models/bowl_concave.urdf"
        bowl_path = "bowl_concave.urdf"
        bowl = p.loadURDF(bowl_path, self._RESET_BOWL_LOC, self._RESET_BOWL_ORI , globalScaling=0.05)

        self.plane = plane
        self.kuka = kuka
        self.gripper = kuka_gripper 
        self.table = table
        self.cube = cube
        self.bowl = bowl

        self.kuka_ee = kuka_ee

        self.reset()

        # gripper constrains 
        kuka_cid = p.createConstraint(kuka, kuka_ee, kuka_gripper, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05],[0, 0, 0])
        pr2_cid2 = p.createConstraint(kuka_gripper,
                                    4,
                                    kuka_gripper,
                                    6,
                                    jointType=p.JOINT_GEAR,
                                    jointAxis=[1, 1, 1],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
        p.changeConstraint(pr2_cid2, gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)

        # enable rendering
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # gravity
        p.setGravity(0, 0, -10)

    def reset(self):
        p.setRealTimeSimulation(0)
        p.setGravity(0, 0, 0)

        # reset kuka
        jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
        for jointIndex in range(p.getNumJoints(self.kuka)):
          p.resetJointState(self.kuka, jointIndex, jointPositions[jointIndex])
          p.setJointMotorControl2(self.kuka, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)

        # reset gripper
        jointPositions = [0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000]
        for jointIndex in range(p.getNumJoints(self.gripper)):
          p.resetJointState(self.gripper, jointIndex, jointPositions[jointIndex])
          p.setJointMotorControl2(self.gripper, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex],0)

        # reset cube
        p.resetBasePositionAndOrientation(self.cube, self._RESET_CUBE_LOC, self._RESET_CUBE_ORI)

        # reset bowl
        p.resetBasePositionAndOrientation(self.bowl, self._RESET_BOWL_LOC, self._RESET_BOWL_ORI)

        p.setGravity(0, 0, -10)

