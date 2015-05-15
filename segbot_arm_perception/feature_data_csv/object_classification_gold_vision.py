print(__doc__)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets, tree
from sklearn.svm import SVC
from scipy.stats import mode

import csv
import numpy
import random
import sys
import collections
import math

# Load a classifier vector based on name (e.g. "red") for each object label
# Note that it includes a column call "ObjectNum"
def get_classifier_vector_list(file_name):
    # Stores the column list for each classifier
    classifier_vector_list = []
    classifier_object_label_list = []
    # Retrieve classifiers for each object
    with open(file_name, "r") as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader, None)
        for row in reader:
            classifier_object_label_list.append(int(row.pop(0)))
            classifier_vector_list.append(np.array(list(row)).astype(np.int32))
            # print(np.array(list(row)).astype(np.int32))

    return (classifier_vector_list,  classifier_object_label_list)


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
    object_label_list = np.squeeze(np.array(list(reader))).astype(np.int32)
    # Read classifier list
    classifier_vector_list, classifier_object_label_list = get_classifier_vector_list("object_classifier_table.csv")

    return (color_feature_vector_list, shape_feature_vector_list, object_label_list,
            classifier_vector_list, classifier_object_label_list)


# Divide the data into train (*_rest) and test (*_sublist)
def subset_feature_vectors_by_object_label(color_feature_vector_list, shape_feature_vector_list,
                                           object_label_list, classifier_vector_list,
                                           classifier_object_label_list,
                                           sublist_object_label_set):
    assert len(color_feature_vector_list) == len(shape_feature_vector_list)
    assert len(object_label_list) == len(color_feature_vector_list)
    assert len(classifier_vector_list) == len(classifier_object_label_list)
    # Size of classifier_vector_list should be the number of unique objects

    color_feature_vector_sublist = []
    shape_feature_vector_sublist = []
    object_label_sublist = []
    classifier_vector_sublist = []
    classifier_object_label_sublist = []
    color_feature_vector_rest = []
    shape_feature_vector_rest = []
    object_label_rest = []
    classifier_vector_rest = []
    classifier_object_label_rest = []

    # Split all data into two bins based on sublist set
    for i, label in enumerate(object_label_list):
        if label in sublist_object_label_set:
            # sublist bin (test)
            color_feature_vector_sublist.append(color_feature_vector_list[i])
            shape_feature_vector_sublist.append(shape_feature_vector_list[i])
            object_label_sublist.append(object_label_list[i])
        else:
            # rest bin (train)
            color_feature_vector_rest.append(color_feature_vector_list[i])
            shape_feature_vector_rest.append(shape_feature_vector_list[i])
            object_label_rest.append(object_label_list[i])

    for i, label in enumerate(classifier_object_label_list):
        if label in sublist_object_label_set:
            classifier_vector_sublist.append(classifier_vector_list[i])
            classifier_object_label_sublist.append(classifier_object_label_list[i])
        else:
            classifier_vector_rest.append(classifier_vector_list[i])
            classifier_object_label_rest.append(classifier_object_label_list[i])

    return (color_feature_vector_sublist, shape_feature_vector_sublist, object_label_sublist, classifier_vector_sublist, classifier_object_label_sublist, color_feature_vector_rest, shape_feature_vector_rest, object_label_rest, classifier_vector_rest, classifier_object_label_rest)



# vision is a dictionary, language parameters are lists
def calculate_accuracy(gold_vector, pred_vector):
    true_positive, false_positive, all_positive = (0, 0, 0)

    assert len(gold_vector) == len(pred_vector)
    # Compare each element of the vectors
    for j in range(len(gold_vector)):
        if gold_vector[j]:
            all_positive += 1
            if pred_vector[j]:
                true_positive += 1
        else:
            if pred_vector[j]:
                false_positive += 1

    # Case when either predicted or
    if (not all_positive) or (not(true_positive + false_positive)):
        precision = float('nan')
        recall = float('nan')
    else:
        precision = true_positive / (true_positive + false_positive)
        recall = true_positive / all_positive

    return (precision, recall)


def knn_classifier(train, label, test, n_neighbors=5):
    # Create model
    # clf = neighbors.KNeighborsClassifier(n_neighbors, weights = 'uniform')
    clf = tree.DecisionTreeClassifier()
    clf.fit(train, label)
    predicted = clf.predict(test)
    # predicted = clf.predict_proba(test)

    return predicted

def svm_classifier(train, label, test, C=1, gamma=1, kernel='rbf'):
    # Create model
    clf = SVC(C=C, gamma=gamma, kernel=kernel)
    clf.fit(train, label)
    predicted = clf.predict(test)

    return predicted


# Collapse each object label into 1 classification
def label_classification_with_object_id(sublist_object_label_set, object_label_sublist, predicted,
                                        expected, cutoff):
    object_classification_predicted = collections.OrderedDict()
    object_classification_expected = collections.OrderedDict()
    # Iterate each object label (column) once in the set
    for object_label in sublist_object_label_set:
        temp_list_predicted = []
        temp_list_expected = []
        # Find all vectors with the same object label
        for i in range(len(object_label_sublist)):
            if object_label == object_label_sublist[i]:
                temp_list_predicted.append(predicted[i])
                temp_list_expected.append(expected[i])
        assert len(temp_list_predicted) != 0
        assert len(temp_list_expected) != 0

        # object_classification_predicted[object_label] = (0 if temp_list_predicted.count(0) > temp_list_predicted.count(1) else 1) # Mode, so above half
        object_classification_predicted[object_label] = (1 if sum(temp_list_predicted)>=cutoff else 0) # Must have more than cutoff number of 1's to be classify as 1
        object_classification_expected[object_label] = (0 if temp_list_expected.count(0) > temp_list_expected.count(1) else 1) # This is always the same value in temp_list_expected

    return (object_classification_predicted, object_classification_expected)


def compute_accuracy(object_classification_predicted, object_classification_expected):
    # Check if correctness against predicted
    object_classification_compare = collections.OrderedDict()
    correct_count = 0
    for key in object_classification_predicted:
        val_0 = object_classification_predicted[key]
        val_1 = object_classification_expected[key]
        object_classification_compare[key] = (True if val_0 == val_1 else False)
        if object_classification_compare[key]:
            correct_count += 1
    # print(object_classification_compare)
    accuracy = correct_count / len(object_classification_compare)
    return accuracy


def calculate_vision_vector_dict(sublist_object_label_set=[], cutoff=6):
    # Input parameters
    color_classifier_list = ["red", "blue", "green", "yellow", "brown", "orange", "pink", "purple", "black", "white", "cylinder", "bowl", "mug", "cap", "plate", "sphere", "cuboid", "car", "animaltoy"]
    n_neighbors = 5
    sublist_ratio = 0.2
    classifier_method = 'knn'
    classifier_kernel = 'poly' # Kernel coefficient for ‘rbf’, ‘poly’ and ‘sigmoid’
    classifier_count = 19 # [color] + [shape]
    color_classifier_index_bound = 9 # exclusive

    random.seed(163)

    # Load needed csv
    color_feature_vector_list, shape_feature_vector_list, object_label_list, classifier_vector_list, classifier_object_label_list = load_files()

    # Get a subset of the features based on object labels
    if not sublist_object_label_set:
        for label in object_label_list:
            if label not in object_label_set:
                object_label_set.append(label)
        sublist_size = int(len(object_label_set) * sublist_ratio)
        sublist_object_label_set = random.sample(object_label_set, sublist_size)
    # # TODO remove this test code
    # sublist_object_label_set = [32]
    sublist_object_label_set.sort()

    # Divide features, object_label, and classifier lists based on the subset
    color_feature_vector_sublist, shape_feature_vector_sublist, object_label_sublist, classifier_vector_sublist, classifier_object_label_sublist, color_feature_vector_rest, shape_feature_vector_rest, object_label_rest, classifier_vector_rest, classifier_object_label_rest = subset_feature_vectors_by_object_label(color_feature_vector_list, shape_feature_vector_list, object_label_list, classifier_vector_list, classifier_object_label_list, sublist_object_label_set)


    # Make a vector for 1 classifier only
    object_classification_vectors = collections.OrderedDict()
    accuracy_list = []
    for class_index in range(classifier_count):
        classifier_label_rest = []
        classifier_label_sublist = []

        # Get classifier value (0 or 1) for each feature vector for train and test
        for label in object_label_rest:
            classifier_label_rest.append(classifier_vector_list[classifier_object_label_list.index(label)][class_index])
        for label in object_label_sublist:
            classifier_label_sublist.append(classifier_vector_list[classifier_object_label_list.index(label)][class_index])

        # Run classifier
        if class_index < color_classifier_index_bound:
            if classifier_method == 'knn':
                predicted = knn_classifier(color_feature_vector_rest, classifier_label_rest,
                                           color_feature_vector_sublist, n_neighbors=n_neighbors)
            if classifier_method == 'svm':
                predicted = svm_classifier(color_feature_vector_rest, classifier_label_rest,
                                           color_feature_vector_sublist, C=1, gamma=1,
                                           kernel=classifier_kernel)
        else:
            if classifier_method == 'knn':
                predicted = knn_classifier(shape_feature_vector_rest, classifier_label_rest,
                                           shape_feature_vector_sublist, n_neighbors=n_neighbors)
            if classifier_method == 'svm':
                predicted = svm_classifier(shape_feature_vector_rest, classifier_label_rest,
                                           shape_feature_vector_sublist, C=1, gamma=1,
                                           kernel=classifier_kernel)

        expected = np.array(classifier_label_sublist)
        objects = np.array(object_label_sublist)
        # print(color_classifier_list[class_index])
        # print(predicted[:])
        # print(expected[:])
        # print()

        # Post-process after getting all the predictions
        # Consolidate data into 1 vector for each object
        # Per column, key = object labe, value = classifier label
        object_classification_predicted, object_classification_expected = label_classification_with_object_id(sublist_object_label_set, object_label_sublist, predicted, expected, cutoff)

        # Put to one list containing all objects
        for label in object_classification_predicted:
            object_classification_vectors.setdefault(label, []).append(object_classification_predicted[label])


    # print("exptected")
    # for i in range(len(object_classification_vectors)):
    #     print("{}: {}".format(sublist_object_label_set[i], classifier_vector_sublist[i].tolist()))

    return object_classification_vectors, sublist_object_label_set, classifier_vector_sublist

def run_experiment():
    # Read language data files
    root_dir = "process_results/"
    n_fold = 5  # number of folds cross validation
    experiment_folder = "{}-fold/".format(n_fold)

    percent_skipped_list = []
    average_precision_list = []
    average_recall_list = []
    # iterate through the cutoff values [0, len=11]
    for cutoff in range(0, 11 + 1):
        # cutoff = 6
        print("cutoff: ", cutoff)
        precision_list = []
        recall_list = []
        for fold_index in range(n_fold):
            # Load files from disk
            reader = csv.reader(open(root_dir + experiment_folder + "run{}/test_index{}.csv".format(fold_index, fold_index), "r"))
            language_sublist_object_label_set = list(np.squeeze(np.array(list(reader))).astype(np.int32))
            language_classifier_vector_list, language_classifier_object_label_list = get_classifier_vector_list(root_dir + experiment_folder + "run{}/spf_bitVector{}.csv".format(fold_index, fold_index))
            assert len(language_classifier_vector_list) == len(language_classifier_object_label_list)
            # This is an ordered dictionary, key as object label, value as vectors
            # key = object label, value = vector containing all class labels
            vision_object_classification_vectors,  sublist_object_label_set, classifier_vector_sublist = calculate_vision_vector_dict(language_sublist_object_label_set, cutoff)

            language_classifier_vector_list = classifier_vector_sublist
            language_classifier_object_label_list = sublist_object_label_set
            # Calculate PR (comparing vision and language vectors)
            total_count, no_match_count = 0, 0
            # Compare vectors of the same object label, iterating over the language vectors
            for i in range(len(language_classifier_object_label_list)):
                label = language_classifier_object_label_list[i]
                vision_vector = vision_object_classification_vectors[label]
                language_vector = language_classifier_vector_list[i]
                precision, recall = calculate_accuracy(language_vector, vision_vector)
                if math.isnan(precision) or math.isnan(recall):
                    no_match_count += 1
                    recall_list.append(0)
                    # precision_list.append(0)
                else:
                    precision_list.append(precision)
                    recall_list.append(recall)
                total_count += 1

        percent_skipped = no_match_count / total_count
        average_precision = sum(precision_list) / len(precision_list)
        average_recall = sum(recall_list) / len(recall_list)
        print("percent skipped: ", percent_skipped)
        print("average precision: ", average_precision)
        print("average recall: ", average_recall)
        percent_skipped_list.append(percent_skipped)
        average_precision_list.append(average_precision)
        average_recall_list.append(average_recall)
    print("\%skip = ", percent_skipped_list)
    print("precis = ", average_precision_list)
    print("recall = ", average_recall_list)
    f1_peak = max([ 2 * tup[0] * tup[1] / (tup[0] + tup[1]) for tup in zip(average_precision_list, average_recall_list) ])

    print("F1 peak = ", f1_peak)
if __name__ == '__main__':
    run_experiment()
