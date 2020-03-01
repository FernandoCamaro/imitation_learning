import torch
from torch import nn, optim
from tensorboardX import SummaryWriter
from torch.utils.data import DataLoader

from environments import kukaEnv
from datasets import KukaStateActionDataset
from models import MLP

# things to define
demonstrations_folder = "demonstrations/cube-to-bowl"
training_steps = int(10e3)
batch_size = 32
controllerID = 4
log_folder = "log"


log = SummaryWriter(log_folder)

model = MLP().cuda()
criteria = nn.MSELoss()
opt = optim.Adam(params=model.parameters(), lr=1e-3)

env = kukaEnv("DIRECT")
dataset = KukaStateActionDataset(demonstrations_folder, env.kuka, controllerID)
dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=0)
loader_iter = iter(dataloader)

for i_batch in range(training_steps):
    batch_state, batch_action = next(loader_iter)
    batch_state = batch_state.cuda()
    batch_action = batch_action.cuda()

    opt.zero_grad()
    y = model(batch_state)
    loss = criteria(y, batch_action)
    loss.backward()
    opt.step()
    log.add_scalar("loss",loss.item(),i_batch)