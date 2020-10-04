"""
This module contains the needed tools to learn  of the neural network.
"""

#----------------------------------------------------------------------- IMPORTS

import keras
import numpy as np
from keras.datasets import mnist
from keras import backend
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D
from keras.utils import plot_model

import sys
sys.path.append('.')
import vis

#----------------------------------------------------------------------- GLOBALS

BATCH_SIZE = 128
CLASSES = [0, 1, 2, 3, 4]

#-------------------------------------------------------------------------- DATA

def load_data():
    """
    This function loads the data and keeps only the classes to use.
    """

    # We retrieve data from keras
    (x_train, y_train), (x_test, y_test) = mnist.load_data()

    # We keep only the right classes
    idx_train = np.isin(y_train, CLASSES)
    idx_test = np.isin(y_test, CLASSES)

    return (x_train[idx_train], y_train[idx_train]),\
        (x_test[idx_test], y_test[idx_test])


def prepare_input(x):
    """
    This function prepares the input. First, it reshapes the array to make it
    compatible with the convolution operation. Second it centers the data
    around 0 with a standard deviation of 1.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def prepare_output(y):
    """
    This function prepares the output.
    """
    ##################
    # YOUR CODE HERE #
    ##################


#------------------------------------------------------------------------- MODEL


def build_model(input_shape, output_shape):
    """
    This function builds the neural network that will be used for
    classification.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def get_loss():
    """
    This function returns the loss to be used during training.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def get_optimizer():
    """
    This function returns the optimizer to be used for the training.
    """
    ##################
    # YOUR CODE HERE #
    ##################


#-------------------------------------------------------------------------- MAIN

if __name__ == "__main__":

    print("1) Loading dataset:")
    print("-------------------")
    (x_train, y_train), (x_test, y_test) = load_data()
    print("x_train:  shape is {}".format(x_train.shape))
    print("          dtype is {}".format(x_train.dtype))
    print("          min is {}".format(x_train.min()))
    print("          max is {}".format(x_train.max()))
    print("y_train:  shape is {}".format(y_train.shape))
    print("          dtype is {}".format(y_train.dtype))
    print("          min is {}".format(y_train.min()))
    print("          max is {}".format(y_train.max()))
    raw_input("Answer questions in 2.1 and press enter to continue...")

    print("2) Previewing raw data")
    print("----------------------")
    vis.preview_samples(x_train, y_train, "Samples")
    raw_input("Answer questions in 2.2 and press enter to continue...")

    print("3) Preparing data")
    print("------------------")
    x_train = prepare_input(x_train)
    x_test = prepare_input(x_test)
    print("x_train:  shape is {}".format(x_train.shape))
    print("          dtype is {}".format(x_train.dtype))
    print("          min is {}".format(x_train.min()))
    print("          max is {}".format(x_train.max()))
    y_train = prepare_output(y_train)
    y_test = prepare_output(y_test)
    print("y_train:  shape is {}".format(y_train.shape))
    print("          dtype is {}".format(y_train.dtype))
    print("          min is {}".format(y_train.min()))
    print("          max is {}".format(y_train.max()))
    raw_input("Answer questions in 2.3 and press enter to continue...")

    print("4) Previewing prepared data")
    print("---------------------------")
    y_train_lab = ["{}".format(a) for a in y_train]
    vis.preview_samples(x_train, y_train_lab, "Samples")
    raw_input("Answer questions in 2.4 and press enter to continue...")

    print("5) Instantiating the model")
    print("--------------------------")
    model = build_model((28, 28, 1), (len(CLASSES)))
    print("Model is: {}".format(model.summary()))
    raw_input("Answer questions in 2.5 and press enter to continue...")

    print("6) Compiling model")
    print("------------------")
    loss = get_loss()
    optimizer = get_optimizer()
    model.compile(loss=loss,
                  optimizer=optimizer,
                  metrics=['accuracy'])
    raw_input("Answer questions in 2.6 and press enter to continue...")

    print("7) Training time !")
    print("------------------")
    model.fit(x_train, y_train,
              batch_size = BATCH_SIZE,
              epochs = 20,
              verbose = 1,
              validation_data = (x_test, y_test))
    raw_input("Answer questions in 2.7 and press enter to continue...")

    print("8) Visualising weights")
    print("----------------------")
    vis.preview_kernels(model.weights[0].numpy(), "First Layer kernels")
    vis.preview_kernels(model.weights[2].numpy(), "Second Layer kernels")
    raw_input("Answer questions in 2.8 and press enter to continue...")

    print("9) Visualising activations")
    print("--------------------------")
    while True:
        inpt = raw_input("Enter the sample index to preview \
        (to skip, just press enter): ")
        if not inpt:
            break
        else:
            idx = int(inpt)
            vis.preview_activations(model, x_train[idx:idx+1], \
                            "Activations of neural network for one sample")

    print("10) Saving the network")
    print("---------------------")
    inpt = raw_input("Enter the path to save the network to \
    (to skip,, just press enter): ")
    if inpt:
        model.save(inpt)
