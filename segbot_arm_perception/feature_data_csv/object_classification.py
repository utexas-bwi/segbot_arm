print(__doc__)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets

import csv
import numpy


def get_classifer_vector(classifier_name, object_label_vector):
    classifier_object_vector = []
    classifier_vector = []
    # Retrieve classifiers for each object
    with open("object_classifier_table.csv", "r") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            classifier_object_vector.append(row[classifier_name])

    # Map each classifier value to each feature
    for object_label in object_label_vector:
        classifier_vector.append(classifier_object_vector[object_label - 1])
    return classifier_vector

if __name__ == '__main__':
    # Input parameters
    classifier_name = 'red'
    n_neighbors = 3

    # Read feature vectors
    reader = csv.reader(open("train.csv", "r"), delimiter=',')
    feature_vector_list = np.array(list(reader))
    feature_vector_list = feature_vector_list.astype(np.float64)
    # Read object labels
    reader = csv.reader(open("train_object_index.csv", "r"))
    object_label_vector = np.array(list(reader))
    object_label_vector = np.squeeze(object_label_vector)
    object_label_vector = object_label_vector.astype(np.int32)
    # Read classifier list
    classifier_vector = np.array(get_classifer_vector(classifier_name, object_label_vector))
    print(feature_vector_list.shape)
    print(object_label_vector.shape)
    print(classifier_vector.shape)

    for i in range(len(feature_vector_list)):
        print("{0}\t{1}".format(object_label_vector[i], classifier_vector[i]))



    # Map feature object pairs

    # Create model
    clf = neighbors.KNeighborsClassifier(n_neighbors, weights = 'uniform')
    clf.fit(feature_vector_list[:][:-11], classifier_vector[:-11])
    prediction = clf.predict(feature_vector_list[:][-11:])
    print("predictions:")
    print(prediction)
    print("expected:")
    print(classifier_vector[-11:])
