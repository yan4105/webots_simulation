import os
from os import path
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
import math

# Notes: two layer with 4->4 and 4->1 works well batch 10
#        three layers with (4,4), (4,3), (4,1)   batch 10

# DEFAULT VARIABLES
UNCER = math.pi / 180
TERRAIN = 0 # mean even
DATAFILE= 'data.txt' if TERRAIN == 1 else 'data_even.txt'
ENSEMBLESIZE = 5
DEBUG=True
SAVE=True

class predictor(nn.Module):
    def __init__(self):
        super(predictor, self).__init__()
        self.fc1 = nn.Linear(4, 4)
        #self.fc2 = nn.Linear(1000, 500)
        #self.fc3 = nn.Linear(500, 250)
        #self.fc4 = nn.Linear(250, 120)
        #self.fc5 = nn.Linear(120, 60)
        #self.fc6 = nn.Linear(4, 3)
        #self.fc7 = nn.Linear(3, 2)
        self.fc8 = nn.Linear(4, 1)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        #x = F.relu(self.fc2(x))
        #x = F.relu(self.fc3(x))
        #x = F.relu(self.fc4(x))
        #x = F.relu(self.fc5(x))
        #x = F.relu(self.fc6(x))
        #x = F.relu(self.fc7(x))
        x = self.fc8(x)
        return x

class dataset(Dataset):
    def __init__(self, file_name, root_dir):
        self.file_name = file_name
        self.__get_data_from_csv()
        self.root_dir = root_dir
        self.IOI = [11,12,13,14]
        self.IOT = [0]

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        #print(idx)
        lst = self.data[idx]
        #print(lst)
        lst = [float(i) for i in lst]
        #for i in self.IOI:
        #    lst[i] = math.cos(lst[i])
        lst = np.array(lst)
        #print(lst)
        return lst[self.IOI], lst[self.IOT]

    def __get_data_from_csv(self):
        with open(self.file_name, newline='') as csvfile:
            self.data = list(csv.reader(csvfile))

def split(ds):
    test = len(ds) // 10
    train = len(ds) - test
    return torch.utils.data.random_split(ds, [train, test])

def train_model(model, trainloader, criterion, max_epoches, thres=1e-3, debug=DEBUG):
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    hl, = plt.plot([], [])
    loss = 0.0
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
            running_loss += loss.item()
        loss = running_loss / len(trainloader)
        if debug: print("train_model: loss is {l}".format(l=loss))
        if loss < thres:
            if debug: print("train_model: training done")
            return

def eval_model(model, ld, uncer=UNCER, debug=DEBUG):
    correct = 0
    total = 0
    with torch.no_grad():
        for data in ld:
            input, labels = data
            input, labels = input[0], labels[0]
            outputs = model(input.float())
            total += 1
            #print(outputs, labels)
            if abs(outputs[0] - labels[0]) < uncer:
                correct += 1
    acc = 100 * correct / total
    if debug: print("eval_model: Accuracy is %d %%" % acc)
    return acc

def ensemble(models, input):
    output = None
    for model in models:
        if output is None: output = model(input)
        else: output += model(input)
    return output / len(models)


def eval_ensemble(uncer=UNCER, debug=DEBUG):
    ds = dataset(DATAFILE, '')
    print("splitting")
    _, test = split(ds)
    ld = DataLoader(test, batch_size=10, shuffle=True, num_workers=1)
    models = load_ensemble()

    correct = 0
    total = 0
    with torch.no_grad():
        for data in ld:
            input, labels = data
            input, labels = input[0], labels[0]
            outputs = ensemble(models, input.float())
            total += 1
            #print(outputs, labels)
            if abs(outputs[0] - labels[0]) < uncer:
                correct += 1
    acc = 100 * correct / total
    if debug: print("Accuracy is %d %%" % acc)
    return acc

def full_train_process(uncer=math.pi/180):
    model = predictor()
    ds = dataset(DATAFILE, '')
    #print("splitting")
    train, test = split(ds)
    train_ld = DataLoader(train, batch_size=10, shuffle=True, num_workers=1)
    test_ld = DataLoader(test, batch_size=10, shuffle=True, num_workers=1)
    #print("training")
    train_model(model, train_ld, criterion=nn.MSELoss(), max_epoches=200, thres=1e-3)
    #print("evaluating")
    return eval_model(model, test_ld, uncer)

def full_train_ensemble(models, uncer=UNCER, save=SAVE):
    for i, model in enumerate(models):
        ds = dataset(DATAFILE, '')
        train, test = split(ds)
        train_ld = DataLoader(train, batch_size=10, shuffle=True, num_workers=1)
        # test_ld = DataLoader(test, batch_size=10, shuffle=True, num_workers=1)
        train_model(model, train_ld, criterion=nn.MSELoss(), max_epoches=200, thres=1e-3)
        if save: torch.save(model.state_dict(), './model_ensemble_{i}.pt'.format(i=i))
    return models

def stability(epoches, uncer=UNCER):
    acc = 0
    for e in range(epoches):
        acc += full_train_process(uncer=uncer)
        print("stability epoch {e}".format(e=e))
    return acc / epoches

def load_ensemble():
    models = []
    if not path.exists("./model_ensemble_0.pt"):
        print("load_ensemble: ERROR, no pretrained weights found")
        return
    i = 0
    while path.exists("./model_ensemble_{i}.pt".format(i=i)):
        model = predictor()
        model.load_state_dict(torch.load("./model_ensemble_{i}.pt".format(i=i)))
        models.append(model)
        i += 1
    return models

def stability_ensemble(epoches, uncer=UNCER, debug=DEBUG):
    acc = 0
    for e in range(epoches):
        models = []
        for _ in range(ENSEMBLESIZE): models.append(predictor())
        full_train_ensemble(models, uncer=uncer)
        acc += eval_ensemble(uncer=uncer, debug=debug)
        print("stability epoch {e}".format(e=e))
    return acc / epoches

def test(ts):
    model = predictor()
    model.load_state_dict(torch.load('./model.pt'))
    for s in ts:
        s = [s]
        o = model(torch.tensor(s).float())
        print(s, o * 180 / math.pi)

if __name__ == '__main__':
    op = 1
    if op == 0:
        model = predictor()
        ds = dataset(DATAFILE, '')
        print("splitting")
        train, test = split(ds)
        train_ld = DataLoader(train, batch_size=10, shuffle=True, num_workers=1)
        test_ld = DataLoader(test, batch_size=10, shuffle=True, num_workers=1)
        print("training")
        train_model(model, train_ld, criterion=nn.MSELoss(), max_epoches=200)
        print("evaluating")
        uncer = 2 * math.pi / 180
        print("Train accuracy is %d %%" % eval_model(model, train_ld, uncer=uncer))
        print("Test accuracy is %d %%" % eval_model(model, test_ld, uncer=uncer))
        torch.save(model.state_dict(), './model.pt')
    elif op == 1: # ensemble learning
        models = []
        for i in range(ENSEMBLESIZE): models.append(predictor())
        models = full_train_ensemble(models)
        eval_ensemble(uncer=2*UNCER)
    elif op == 2:
        print("stability is: ", stability(20, uncer=5*UNCER))
    elif op == 3: # ensemble stability
        print("emsemble stability is: ", stability_ensemble(20))
    elif op == 4: # eval_ensemble using pretrained weights
        print("ensemble accuracy is: ", eval_ensemble())
    else:
        test([[0.4, 0.4, 0.4, 0.4]])
