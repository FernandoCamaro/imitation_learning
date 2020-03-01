from torch import nn

class MLP(nn.Module):
        def __init__(self):
                super(MLP, self).__init__()
                self.fc1 = nn.Linear(3,32)
                self.fc2 = nn.Linear(32,64)
                self.fc3 = nn.Linear(64,3)
                self.act = nn.ReLU(inplace=True)
        def forward(self,x):
                x = self.act(self.fc1(x))
                x = self.act(self.fc2(x))
                y = self.fc3(x)
                return y