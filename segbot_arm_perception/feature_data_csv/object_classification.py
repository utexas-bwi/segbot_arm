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
    reader = csv.reader(open("train.csv", "r"), delimiter=',')
    X = np.array(list(reader))
    X = X.astype(np.float64)
    # Read object labels
    reader = csv.reader(open("train_object_index.csv", "r"))
    y = np.array(list(reader))
    # y = y.astype(np.int64)
    y = np.squeeze(y)
    print(X.shape)
    print(y.shape)
    # Read classifier list
    # with open("object_classifier_table.csv", "r") as csvfile:
    #     reader = csv
    reader = csv.reader(open("object_classifiers.csv", "r"), delimiter=',')
    classifier_table = np.array(list(reader))

    # Map feature object pairs

    # Create model
    clf = neighbors.KNeighborsClassifier(n_neighbors, weights = 'uniform')
    clf.fit(X[:][:-11], y[:-11])
    prediction = clf.predict(X[:][-11:])
    print("predictions:")
    print(prediction)
    print("expected:")
    print(y[-11:])
