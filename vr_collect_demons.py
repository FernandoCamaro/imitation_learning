import pybullet as p
import numpy as np

from environments import kukaEnv

# controller and buttons
controller_id = 3
POSITION = 1
ORIENTATION = 2
ANALOG = 3
BUTTONS = 6

import math
LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
JOINT_DAMP = [0.1] * 10
MAX_FORCE = 500

env = kukaEnv("SHARED_MEMORY")

doing_demons = False
demons_id = 9 # starting demonstration id

while True:
  events = p.getVREvents()
  for e in (events):
    if e[0] == controller_id:
      break
    else:

      just_finished = False

      if doing_demons and (e[BUTTONS][7] & p.VR_BUTTON_WAS_RELEASED):

        # stop demonstrations and increase id for next one
        print("end demonstration")
        doing_demons = False
        p.stopStateLogging(logId_gen)
        p.stopStateLogging(logId_vr)
        demons_id += 1
        just_finished = True

        # reset all elements
        p.setRealTimeSimulation(0)
        p.setGravity(0, 0, 0)

        # reset env
        env.reset()
        
        p.setRealTimeSimulation(1)
        

      if doing_demons or (e[BUTTONS][7] & p.VR_BUTTON_WAS_RELEASED and (not just_finished)):

        # start demons
        if doing_demons == False:
          doing_demons = True
          print("new demonstration")
          p.setRealTimeSimulation(0)
          logId_gen = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "LOG_GENERIC_"+str(demons_id)+".bin",[env.kuka, env.gripper, env.cube, env.bowl])
          logId_vr  = p.startStateLogging(p.STATE_LOGGING_VR_CONTROLLERS, "LOG_VR_"+str(demons_id)+".bin")
          p.setRealTimeSimulation(1)
          start_kuka = np.array(p.getLinkState(env.kuka, 6)[0])
          start_cont = np.array(e[POSITION])
          start_M = np.stack((start_kuka,start_cont))
          np.savetxt("start_locs_"+str(demons_id)+".txt", start_M) 
        
        # gripper control  
        for i in [4,6]:
          p.setJointMotorControl2(env.gripper, i, p.POSITION_CONTROL, targetPosition=e[ANALOG] * 0.05, force=10)

        # kuka control                
        eef_pos = (np.array(e[POSITION]) - start_cont) + start_kuka
        # print(eef_pos)
        eef_orn = p.getQuaternionFromEuler([0, -math.pi, 0])
        joint_pos = p.calculateInverseKinematics(env.kuka, 6, list(eef_pos), eef_orn, lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS, jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)

        for i in range(len(joint_pos)):
          p.setJointMotorControl2(env.kuka,
                                  i,
                                  p.POSITION_CONTROL,
                                  targetPosition=joint_pos[i],
                                  targetVelocity=0,
                                  positionGain=0.15,
                                  velocityGain=1.0,
                                  force=MAX_FORCE)
