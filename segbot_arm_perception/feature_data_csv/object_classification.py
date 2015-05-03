print(__doc__)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets

import csv
import numpy

if __name__ == '__main__':
    n_neighbors = 3

    # Read feature vectors
    reader = csv.reader(open("train.csv","r"), delimiter=',')
    X = np.array(list(reader))
    X = X.astype(np.float64)
    # Read labels
    reader = csv.reader(open("train_object_index.csv"))
    y = np.array(list(reader))
    # y = y.astype(np.int64)
    y = np.squeeze(y)
    print(X.shape)
    print(y.shape)
    # y_2 = y[0, :]  # Transpose
    # y_3 = np.squeeze(y)
    # print(X.size)
    # print(y_2.size)
    # print(y_2)
    # print(y_3.size)
    # print(y_3)
    # iris = datasets.load_iris()
    # X_i = iris.data[:, :2]  # we only take the first two features. We could
    # # avoid this ugly slicing by using a two-dim dataset
    # y_i = iris.target
    # print(X_i.size)
    # print(y_i.size)
    # print(y_i)
    # print(iris.data)
    # print(iris.data[:, :2])
    # print(X[:][:-11])
    # print(y[:][0])

    # Create model
    clf = neighbors.KNeighborsClassifier(n_neighbors, weights = 'uniform')
    clf.fit(X[:][:-11], y[:-11])
    prediction = clf.predict(X[:][-11:])
    print("predictions:")
    print(prediction)
    print("expected:")
    print(y[-11:])
