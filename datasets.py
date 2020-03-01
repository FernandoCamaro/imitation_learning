import torch
import pybullet as p
from torch.utils.data import Dataset
import glob
import os.path as path
import numpy as np
import random

from readlogfile import readLogFile
from utils import reset_state


class KukaStateActionDataset(Dataset):
        def __init__(self, folder, kuka_id, controller_id):

                vr_files = glob.glob(path.join(folder,'LOG_VR_*.bin'))
                log_gens = []
                log_vrs = []
                start_kuka_locs = []
                start_cont_locs = []
                for vr_file in vr_files:
                        generic_file = vr_file.replace('VR','GENERIC')
                        log_gen = readLogFile(generic_file, verbose=False)
                        log_vr = readLogFile(vr_file, verbose=False)
                        demons_id = generic_file.split('_')[-1].split('.')[0]
                        start_locs_file = "start_locs_"+demons_id+'.txt'
                        A = np.loadtxt(path.join(folder,start_locs_file))
                        log_gens.append(log_gen)
                        log_vrs.append(log_vr)
                        start_kuka_locs.append(A[0])
                        start_cont_locs.append(A[1])

                
                self.log_vrs = log_vrs
                self.log_gens = log_gens
                self.start_kuka_locs = start_kuka_locs
                self.start_cont_locs = start_cont_locs
                self.kuka_id = kuka_id
                self.controller_id = controller_id
                self.kuka_ee = 6

        def __len__(self):

                return int(32*1e4)#len(self.log_vr)

        def __getitem__(self, idx):

                demons_idx = random.randint(0,len(self.log_vrs)-1)
                self.log_vr = self.log_vrs[demons_idx]
                self.log_gen = self.log_gens[demons_idx]

                step_count_in_both_log = False
                while not step_count_in_both_log:
                        step_count = random.sample(self.log_vr.keys(),1)[0]
                        if step_count in self.log_gen.keys() and self.controller_id in self.log_vr[step_count].keys() and self.kuka_id in self.log_gen[step_count]:
                                step_count_in_both_log = True

                kuka_start_state = self.start_kuka_locs[demons_idx]
                cont_start_loc = self.start_cont_locs[demons_idx]

                kuka_state = self.log_gen[step_count][self.kuka_id]
                reset_state(self.kuka_id, kuka_state)
                kuka_state = np.array(p.getLinkState(self.kuka_id, self.kuka_ee)[0])
                cont_state = self.log_vr[step_count][self.controller_id]
                cont_loc = np.array([cont_state['posX'],cont_state['posY'],cont_state['posZ']])
                action = (cont_loc-cont_start_loc+kuka_start_state) - kuka_state

                return torch.Tensor(kuka_state), torch.Tensor(action)

# # example
# from environments import kukaEnv
# controllerID = 4
# env = kukaEnv()
# dataset = KukaStateActionDataset("demonstrations/cube-to-bowl", env.kuka, controllerID)
# dataset.__getitem__(0)