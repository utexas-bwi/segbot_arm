print(__doc__)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets
from scipy.stats import mode

import csv
import numpy
import random
import sys


# Load a classifier vector based on name (e.g. "red") for each object label
# Note that it includes a column call "ObjectNum"
def get_classifier_vector_list():
    # Stores the column list for each classifier
    classifier_vector_list = []
    object_label_list = []
    # Retrieve classifiers for each object
    with open("object_classifier_table.csv", "r") as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader, None)
        for row in reader:
            object_label_list.append(row.pop(0))
            classifier_vector_list.append(np.array(list(row)).astype(np.int32))
            print(np.array(list(row)).astype(np.int32))

        print( object_label_list)


    # # Map each classifier value to each feature
    # for object_label in object_label_vector:
    #     classifier_vector.append(classifier_object_vector[object_label - 1])

    return (classifier_vector_list,  object_label_list)


# Divide the data into train (*_rest) and test (*_sublist)
def subset_feature_vectors_by_object_label(color_feature_vector_list, shape_feature_vector_list,
                                           object_label_list, classifier_vector_list,
                                           sublist_object_label_set):
    feature_vector_subset_index_list = []
    feature_vector_rest_index_list = []
    color_feature_vector_sublist = []
    shape_feature_vector_sublist = []
    object_label_sublist = []
    classifier_vector_sublist = []
    color_feature_vector_rest = []
    shape_feature_vector_rest = []
    object_label_rest = []
    classifier_vector_rest = []

    # Get find all the feature indices for desired objects
    for i in range(len(object_label_list)):
        if object_label_list[i] in sublist_object_label_set:
            feature_vector_subset_index_list.append(i)
        else:
            feature_vector_rest_index_list.append(i)

    # Construct new sublists
    for i in feature_vector_subset_index_list:
        color_feature_vector_sublist.append(color_feature_vector_list[i])
        shape_feature_vector_sublist.append(shape_feature_vector_list[i])
        object_label_sublist.append(object_label_list[i])
        classifier_vector_sublist.append(classifier_vector_list[i])
    for i in feature_vector_rest_index_list:
        color_feature_vector_rest.append(color_feature_vector_list[i])
        shape_feature_vector_rest.append(shape_feature_vector_list[i])
        object_label_rest.append(object_label_list[i])
        classifier_vector_rest.append(classifier_vector_list[i])

    return (color_feature_vector_sublist, shape_feature_vector_sublist, object_label_sublist, classifier_vector_sublist, color_feature_vector_rest, shape_feature_vector_rest, object_label_rest, classifier_vector_rest)

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
    if all_positive == 0 or false_positive == 0:
        recall = true_positive / all_positive
        precision = true_positive / (true_positive + false_positive)
    else:
        recall = float("nan")
        precision = float("nan")

    return (recall, precision)


# Load all relevant .csv files to disk
def load_files():
     # Read feature vectors
    reader = csv.reader(open("color_feature_vectors.csv", "r"), delimiter=',')
    color_feature_vector_list = np.array(list(reader))
    color_feature_vector_list = color_feature_vector_list.astype(np.float64)
    reader = csv.reader(open("cvfh_feature_vectors.csv", "r"), delimiter=',')
    shape_feature_vector_list = np.array(list(reader))
    shape_feature_vector_list = shape_feature_vector_list.astype(np.float64)
    # Read object labels (object index starts from 1)
    reader = csv.reader(open("cvfh_feature_object_index.csv", "r"))
    object_label_vector = np.array(list(reader))
    object_label_vector = np.squeeze(object_label_vector)
    object_label_vector = object_label_vector.astype(np.int32)
    # Read classifier list
    classifier_vector_list, object_label_list = get_classifier_vector_list()
    # Extract "ObjectNum" vector for mapping classifier index to object number
    print(color_feature_vector_list.shape)
    print(shape_feature_vector_list.shape)
    print(object_label_vector.shape)

    return (color_feature_vector_list, shape_feature_vector_list, object_label_list,
            classifier_vector_list)

def knn_classifier(n_neighbors, train, label, test):
    # Create model
    clf = neighbors.KNeighborsClassifier(n_neighbors, weights = 'uniform')
    clf.fit(train, label)
    predicted = clf.predict(test)
    # predicted = clf.predict(feature_vector_sublist)
    # expected = np.array(classifier_sublist)
    # objects = np.array(object_label_sublist)
    # recall, precision = calculate_accuracy(predicted, expected)
    # print("predictions:")
    # print(predicted)
    # print("expected:")
    # print(expected)
    # print("objects:")
    # print(objects)
    # sys.stdout.flush()
    # print("recall: {0} \t precision: {1} \t F1: {2}".format(recall, precision,
    #                                                         2*recall*precision/(recall+precision)))

    return predicted

def main():
    # Input parameters
    random.seed(163)
    color_classifier_list = ["red", "blue", "green", "yellow", "brown", "orange", "pink", "purple", "black", "white"]
    shape_classifier_list = ["cylinder", "bowl", "mug", "cap", "plate", "sphere", "cuboid", "car", "animaltoy"]
    classifier_name = 'cylinder'
    n_neighbors = 5
    sublist_ratio = 0.2

    # Load needed csv
    color_feature_vector_list, shape_feature_vector_list, object_label_list, classifier_vector_list = load_files()

    # Get a subset of the features based on object labels
    object_label_set = set(object_label_list)
    sublist_size = int(len(object_label_set) * sublist_ratio)
    sublist_object_label_set = random.sample(object_label_set, sublist_size)

    color_feature_vector_sublist, shape_feature_vector_sublist, object_label_sublist, classifier_sublist, color_feature_vector_rest, shape_feature_vector_rest, object_label_rest, classifier_rest = subset_feature_vectors_by_object_label(color_feature_vector_list, shape_feature_vector_list, object_label_list, classifier_vector_list, sublist_object_label_set)

    # TODO second argument should be label

    class_index = 3
    classifier_labels = [ classifier_vector[class_index] for classifier_vector in classifier_rest ]

    predicted = knn_classifier(n_neighbors, color_feature_vector_rest, classifier_labels, color_feature_vector_sublist)
    expected = np.array([ classifier_vector[class_index] for classifier_vector in classifier_sublist ])
    objects = np.array(object_label_sublist)

    print("predictions:")
    print(predicted)
    print("expected:")
    print(expected)
    print("objects:")
    print(objects)
    # print("recall: {0} \t precision: {1} \t F1: {2}".format(recall, precision,
    #                                                         2*recall*precision/(recall+precision)))

    # Collapse each object label into 1 classification
    object_classification_predicted = {}
    object_classification_expected = {}
    object_classification_compare = {}
    print(sublist_object_label_set)
    for object_label in sublist_object_label_set:
        temp_list_predicted = []
        temp_list_expected = []
        for i in range(len(object_label_sublist)):
            if object_label == object_label_sublist[i]:
                temp_list_predicted.append(predicted[i])
                temp_list_expected.append(expected[i])
        assert len(temp_list_predicted) != 0
        assert len(temp_list_expected) != 0
        # Find mode
        object_classification_predicted[object_label] = (0 if temp_list_predicted.count(0) > temp_list_predicted.count(1) else 1)
        object_classification_expected[object_label] = (0 if temp_list_expected.count(0) > temp_list_expected.count(1) else 1)
    print(object_classification_predicted)
    print(object_classification_expected)
    for key in object_classification_predicted:
        val_0 = object_classification_predicted[key]
        val_1 = object_classification_expected[key]
        object_classification_compare[key] = (True if val_0 == val_1 else False)
    print(object_classification_compare)

if __name__ == '__main__':
    main()
