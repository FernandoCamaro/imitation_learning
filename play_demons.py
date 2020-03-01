
import pybullet as p
from time import sleep

from readlogfile import readLogFile
from environments import kukaEnv
from utils import reset_state

env = kukaEnv()
log = readLogFile("demonstrations/cube-to-bowl/LOG_GENERIC_1.bin", verbose=False)
recordNum = len(log)
itemNum = len(log[0])

for step, record in log.items():
  for k, elem in record.items():
    if type(elem) == dict and "objectId" in elem.keys(): # only the objects have integer keys
      Id = elem["objectId"]
      reset_state(Id, elem)
      
  sleep(0.001)