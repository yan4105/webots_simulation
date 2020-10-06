import torch
import torch.nn as nn
import torch.nn.functional as F

class predictor(nn.Module):
    def __init__(self):
        super(predictor, self).__init__()
        self.fc1 = nn.Linear(12, 8)
        self.fc2 = nn.Linear(8, 5)
        self.fcx = nn.Linear(5, 3)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
