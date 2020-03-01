import pybullet as p

def reset_state(object_id, state):
    pos = [state['posX'], state['posY'], state['posZ']]
    orn = [state['oriX'], state['oriY'], state['oriZ'], state['oriW']]
    p.resetBasePositionAndOrientation(object_id, pos, orn)
    numJoints = p.getNumJoints(object_id)
    for i in range(numJoints):
        jointInfo = p.getJointInfo(object_id, i)
        qIndex = jointInfo[3]
        if qIndex > -1:
            p.resetJointState(object_id, i, state["q"+str(qIndex-7)])