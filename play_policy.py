import pybullet as p
import torch

from environments import kukaEnv
from models import MLP
import numpy as np

# things to be define
num_steps = 5000
checkpoint_path = "trained_models/model_cube_bowl.pth"

# this should be in the environment....
import math
LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
JOINT_DAMP = [0.1] * 10
MAX_FORCE = 500

env = kukaEnv()
model = MLP().cuda()
checkpoint = torch.load(checkpoint_path)
model.load_state_dict(checkpoint)
model.eval()

input("Press enter to continue...")
p.setRealTimeSimulation(1)
step = 0
while (step<num_steps):
  step +=1
  kuka_state = np.array(p.getLinkState(env.kuka, env.kuka_ee)[0])
  action = model(torch.Tensor(kuka_state).unsqueeze(0).cuda())[0].detach().cpu().numpy()
  eef_pos = kuka_state + action
  eef_orn = p.getQuaternionFromEuler([0, -math.pi, 0])
  joint_pos = p.calculateInverseKinematics(env.kuka, env.kuka_ee, eef_pos, eef_orn, lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS, jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)
  for i in range(len(joint_pos)):
          p.setJointMotorControl2(env.kuka,
                                  i,
                                  p.POSITION_CONTROL,
                                  targetPosition=joint_pos[i],
                                  targetVelocity=0,
                                  positionGain=0.15,
                                  velocityGain=1.0,
                                  force=MAX_FORCE)