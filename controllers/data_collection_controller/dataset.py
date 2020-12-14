import os
import torch
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import csv
from torch.utils.data import Dataset, DataLoader
import torch
from torch import nn
import torch.nn.functional as F
import torch.optim as optim

class predictor(nn.Module):
    def __init__(self):
        super(predictor, self).__init__()
        self.fc1 = nn.Linear(12, 9)
        #self.fc2 = nn.Linear(9, 8)
        #self.fc3 = nn.Linear(8, 5)
        self.fc4 = nn.Linear(9, 3)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        #x = F.relu(self.fc2(x))
        #x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x

class dataset(Dataset):
    def __init__(self, file_name, root_dir):
        self.file_name = file_name
        self.__get_data_from_csv()
        self.root_dir = root_dir
        self.IOI = [3,4,5,6]
        self.IOT = [0,1,2]

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        lst = self.data[idx]
        lst = [float(i) for i in lst]
        lst = np.array(lst)
        return lst[self.IOI], lst[self.IOT]

    def __get_data_from_csv(self):
        with open(self.file_name, newline='') as csvfile:
            self.data = list(csv.reader(csvfile))

def split(ds):
    test = len(ds) // 10
    train = len(ds) - test
    return torch.utils.data.random_split(ds, [train, test])

def train_model(model, trainloader, criterion, max_epoches):
    optimizer = optim.Adam(model.parameters(), lr=0.01)
    hl, = plt.plot([], [])
    for epoch in range(max_epoches):
        running_loss = 0.0
        for i, data in enumerate(trainloader, 0):
            inputs, labels = data
            inputs, labels = inputs[0], labels[0]
            optimizer.zero_grad()
            outputs = model(inputs.float())
            loss = criterion(outputs, labels.float())
            loss.backward()
            optimizer.step()
            running_loss = loss.item()
            print(running_loss)

def eval_model(model, ld):
    correct = 0
    total = 0
    with torch.no_grad():
        for data in ld:
            input, labels = data
            input, labels = input[0], labels[0]
            outputs = model(input.float())
            total += labels.size(0)
            print(outputs, labels)
            if (outputs[0] - labels[0]) < 0.01:
                correct += 3
    print("Accuracy is %d %%" % (100 * correct / total))

if __name__ == '__main__':
    model = predictor()
    ds = dataset('data.txt', '')
    train, test = split(ds)
    train_ld = DataLoader(train, batch_size=1, shuffle=True, num_workers=1)
    test_ld = DataLoader(test, batch_size=1, shuffle=True, num_workers=1)
    train_model(model, train_ld, criterion=nn.MSELoss(), max_epoches=1)
    eval_model(model, train_ld)
