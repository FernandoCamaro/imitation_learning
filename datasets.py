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


class TCNDataset(Dataset):
        def __init__(self, folder):
                gen_files = glob.glob(path.join(folder,'LOG_GENERIC_*.bin'))
                log_gens = []
                for gen_file in gen_files:
                        log_gen = readLogFile(gen_file, verbose=False)
                        log_gens.append(log_gen)
                self.log_gens = log_gens
                self.num_perspectives = 3
                self.eye_locs = [[0,0,.7],[0,0,.9],[0,0,1.1]] # TODO: I prefer cameras in a circunference centered to where the cube is, for example...
                self.target_loc = [0.95, -0.1, 0.65] #TODO
                self.up_vector = [0,0,1]
                self.proj_matrix = p.computeProjectionMatrixFOV(60,1,0.1,3) # TODO
                self.width = 512
                self.height = 512

        def __len__(self):
                return int(32*1e4)
        
        def __getitem__(self, idx):
                # sample a demonstration
                demons_idx = random.randint(0,len(self.log_gens)-1)
                log_gen = self.log_gens[demons_idx]
                
                # sample a two different time steps from the demonstration
                t_ap, t_n = random.sample(list(np.arange(len(log_gen))),2)
                
                # render two different perspectives of one time step (anchor and positive images), and another perspective the other time step (negative image)
                # sample the perspectives
                pers_a, pers_p = random.sample(list(np.arange(self.num_perspectives)), 2)
                pers_n = random.sample(list(np.arange(self.num_perspectives)), 1)[0]
                
                # render the anchor, the positive and the negative images
                images = []
                for t, pers in zip([t_ap, t_ap, t_n], [pers_a, pers_p, pers_n]):
                        # view matrix
                        view_matrix = p.computeViewMatrix(self.eye_locs[pers], self.target_loc, self.up_vector)

                        # set the environment to the required state
                        record = log_gen[t]
                        for k, elem in record.items():
                                if type(elem) == dict and "objectId" in elem.keys(): # only the objects have integer keys
                                        Id = elem["objectId"]
                                        reset_state(Id, elem)
                        
                        # render the image
                        out = p.getCameraImage(self.width, self.height, view_matrix, self.proj_matrix)
                        rgba = out[2]
                        print("shape", rgba.shape)
                        images.append((rgba[:,:,0:3]-128.).transpose((2, 0, 1)) / 128.0)


                return images[0], images[1], images[2]
    
        def _ppi(self, image): # post process image
                image = np.array(image, np.float32)
                image = (image-128.).transpose((2, 0, 1)) / 128.0
                return image

# # example for KukaStateActionDataset
# from environments import kukaEnv
# controllerID = 4
# env = kukaEnv()
# dataset = KukaStateActionDataset("demonstrations/cube-to-bowl", env.kuka, controllerID)
# dataset.__getitem__(0)

# example for TCNDataset
from PIL import Image
from environments import kukaEnv
env = kukaEnv()
dataset = TCNDataset("demonstrations/cube-to-bowl")
anchor, positive, negative = dataset.__getitem__(0)

Image.fromarray((anchor*128+128).astype(np.uint8).transpose(1,2,0)).show()
Image.fromarray((positive*128+128).astype(np.uint8).transpose(1,2,0)).show()
Image.fromarray((negative*128+128).astype(np.uint8).transpose(1,2,0)).show()