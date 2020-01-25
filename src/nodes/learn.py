#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Training of neural network
"""
import keras
import numpy as np
import ros4pro.vision.vis as vis
from keras.datasets import mnist
from keras import backend
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D
from keras.utils import plot_model
from os.path import join
from rospkg.rospack import RosPack

BATCH_SIZE=128
CLASSES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

rospack = RosPack()
path = rospack.get_path('ros4pro')

def load_data():
    """
    This function loads the data and keeps only the classes to use.
    """

    # We retrieve data from keras
    (x_train, y_train), (x_test, y_test) = mnist.load_data()

    # We keep only the right classes
    idx_train = np.isin(y_train, CLASSES)
    idx_test = np.isin(y_test, CLASSES)

    return (x_train[idx_train], y_train[idx_train]), (x_test[idx_test], y_test[idx_test])


def prepare_input(x):
    """
    This function prepares the input.
    """

    # First transformation
    x = x.reshape(-1, 28, 28, 1)

    # Second transformation
    x = x / x.std()
    x = x - x.mean()

    return x


def prepare_output(y):
    """
    This function prepares the output.
    """

    # First transformation
    for i, c in enumerate(CLASSES):
        y[y == c] = i

    # Second transformation
    y = keras.utils.to_categorical(y, len(CLASSES))

    return y

def build_model(input_shape, output_shape):
    """
    This function buildes the neural network that will be used for classification.
    """

    model = Sequential()

    model.add(Conv2D(6, kernel_size=(3, 3), activation='relu', input_shape=input_shape))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Conv2D(16, kernel_size=(5, 5), activation='relu', input_shape=input_shape))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Flatten())
    model.add(Dense(120, activation='relu'))
    model.add(Dense(84, activation='relu'))
    model.add(Dense(output_shape, activation='softmax'))
    return model

if __name__ == "__main__":
    print("1) Loading dataset:")
    print("-------------------")
    (x_train, y_train), (x_test, y_test) = load_data()
    print("x_train is a numpy array of shape is {}".format(x_train.shape))
    print("y_train is a numpy array of shape is {}".format(y_train.shape))
    raw_input("Press enter to continue...")

    print("2) Showing a few samples")
    print("------------------------")
    vis.preview_samples(x_train, y_train, "Samples")
    raw_input("Press enter to continue...")

    print("3) Preparing data")
    print("------------------")
    x_train = prepare_input(x_train)
    x_test = prepare_input(x_test)
    y_train = prepare_output(y_train)
    y_test = prepare_output(y_test)
    print("x_train is a numpy array of shape is {}".format(x_train.shape))
    print("y_train is a numpy array of shape is {}".format(y_train.shape))
    raw_input("Press enter to continue...")

    print("4) Instantiating the model")
    print("--------------------------")
    model = build_model((28, 28, 1), (len(CLASSES)))
    print("Model is: {}".format(model.summary()))
    raw_input("Press enter to continue...")

    print("5) Compiling model")
    print("------------------")
    loss = keras.losses.categorical_crossentropy
    optimizer = keras.optimizers.Adam()
    model.compile(loss=loss,
                optimizer=optimizer,
                metrics=['accuracy'])
    raw_input("Press enter to continue...")

    print("6) Training time !")
    print("------------------")
    model.fit(x_train, y_train,
            batch_size = BATCH_SIZE,
            epochs = 1,
            verbose = 1,
            validation_data = (x_test, y_test))
    raw_input("Press enter to continue...")

    # FIXME
    #print("7) Visualising weights")
    #print("----------------------")
    #vis.preview_kernels(tf.Tensor().numpy(model.weights[0]), "First Layer kernels")
    #vis.preview_kernels(tf.Tensor().numpy(model.weights[2]), "Second Layer kernels")
    #raw_input("Press enter to continue...")

    print("8) Visualising activations")
    print("--------------------------")
    while True:
        inpt = raw_input("Enter the sample index to preview (to skip, just press enter): ")
        if not inpt:
            break
        else:
            idx = int(inpt)
            vis.preview_activations(model, x_train[idx:idx+1], "Activations of neural network for one sample")

    print("9) Saving the network")
    print("---------------------")
    file = join(path, "checkpoints", "checkpoint")
    model.save(file)
    print(file)
