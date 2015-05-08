print(__doc__)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets

import csv
import numpy
import random
import sys

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

# Subset object labels are the object number for
def subset_feature_vectors_by_object_label(feature_vector_list, object_label_vector,
                                           classifier_vector, subset_object_labels):
    feature_vector_subset_index_list = []
    feature_vector_rest_index_list = []
    feature_vector_sublist = []
    object_label_sublist = []
    classifier_sublist = []
    feature_vector_rest = []
    object_label_rest = []
    classifier_rest = []

    # Get find all the feature indices for desired objects
    for i in range(len(object_label_vector)):
        if object_label_vector[i] in subset_object_labels:
            feature_vector_subset_index_list.append(i)
        else:
            feature_vector_rest_index_list.append(i)

    # Construct new sublists
    for i in feature_vector_subset_index_list:
        feature_vector_sublist.append(feature_vector_list[i])
        object_label_sublist.append(object_label_vector[i])
        classifier_sublist.append(classifier_vector[i])
    for i in feature_vector_rest_index_list:
        feature_vector_rest.append(feature_vector_list[i])
        object_label_rest.append(object_label_vector[i])
        classifier_rest.append(classifier_vector[i])

    return (feature_vector_sublist, object_label_sublist, classifier_sublist,
            feature_vector_rest, object_label_rest, classifier_rest)

def calculate_accuracy(predicted, expected):
    true_positive, false_positive, all_positive = 0, 0, 0
    # recall, precision = 0.0001, 0
    for i in range(expected.size):
        if predicted[i]:
            if expected[i]:
                true_positive += 1
            else:
                false_positive += 1
        if expected[i]:
            all_positive += 1

    # TODO avoid /0 case that causes the program to crash
    recall = true_positive / all_positive
    precision = true_positive / (true_positive + false_positive)

    return (recall, precision)


if __name__ == '__main__':
    # Input parameters
    # random.seed(163)
    classifier_name = 'blue'
    n_neighbors = 5
    sublist_ratio = 0.2

    # Read feature vectors
    reader = csv.reader(open("train.csv", "r"), delimiter=',')
    feature_vector_list = np.array(list(reader))
    feature_vector_list = feature_vector_list.astype(np.float64)
    # Read object labels (object index starts from 1)
    reader = csv.reader(open("train_object_index.csv", "r"))
    object_label_vector = np.array(list(reader))
    object_label_vector = np.squeeze(object_label_vector)
    object_label_vector = object_label_vector.astype(np.int32)
    # Read classifier list
    classifier_vector = np.array(get_classifer_vector(classifier_name, object_label_vector))
    classifier_vector = classifier_vector.astype(np.int32)
    print(feature_vector_list.shape)
    print(object_label_vector.shape)
    print(classifier_vector.shape)
    assert (feature_vector_list.shape[0] == object_label_vector.size)
    assert (object_label_vector.size == classifier_vector.size)

    # Get a subset of the features based on object
    object_label_set = set(object_label_vector)
    sublist_size = int(len(object_label_set) * sublist_ratio)
    sublist_object_label_set = random.sample(object_label_set, sublist_size)
    feature_vector_sublist, object_label_sublist, classifier_sublist, feature_vector_rest, object_label_rest, classifier_rest = subset_feature_vectors_by_object_label(feature_vector_list, object_label_vector, classifier_vector, sublist_object_label_set)

    # for i in range(len(feature_vector_list)):
    #     print("{0}\t{1}".format(object_label_vector[i], classifier_vector[i]))

    # Create model
    clf = neighbors.KNeighborsClassifier(n_neighbors, weights = 'uniform')
    # clf.fit(feature_vector_list[:][:-11], classifier_vector[:-11])
    # prediction = clf.predict(feature_vector_list[:][-11:])
    clf.fit(feature_vector_rest, classifier_rest)
    predicted = clf.predict(feature_vector_sublist)
    expected = np.array(classifier_sublist)
    objects = np.array(object_label_sublist)
    recall, precision = calculate_accuracy(predicted, expected)
    print("predictions:")
    print(predicted)
    print("expected:")
    print(expected)
    print("objects:")
    print(objects)
    sys.stdout.flush()
    print("recall: {0} \t precision: {1} \t F1: {2}".format(recall, precision,
                                                            2*recall*precision/(recall+precision)))
