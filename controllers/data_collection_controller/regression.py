import numpy as np
from sklearn.multioutput import MultiOutputRegressor
from sklearn.linear_model import Ridge
from sklearn import svm
from torch.utils.data import Dataset
import torch
import csv

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
        #print(idx)
        lst = self.data[idx]
        #print(lst)
        lst = [float(i) for i in lst]
        lst = np.array(lst)
        #print(lst)
        return lst[self.IOI], lst[self.IOT]

    def __get_data_from_csv(self):
        with open(self.file_name, newline='') as csvfile:
            self.data = list(csv.reader(csvfile))

def loadXy(dataset):
    X, y = [], []
    for i in range(len(dataset)):
        x, t = dataset[i]
        #x, t = x.reshape(-1, 1), t.reshape(-1, 1)
        X.append(x)
        y.append(t)
    return X, y

if __name__ == "__main__":
    ds = dataset('data.txt', '')
    X, y = loadXy(ds)
    #clf = MultiOutputRegressor(Ridge(random_state=123)).fit(X, y)
    clf =
    print(X[0])
    print(clf.predict([X[0]]))