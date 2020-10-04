"""
Visualization using visdom.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from itertools import *
from keras import backend

def preview_samples(images, labels, title):
    """
    Shows a few samples of the loader.
    """

    # We generate a plot
    (fig, ax) = plt.subplots(11, 11, figsize=(10, 10))
    plt.subplots_adjust(hspace=0.5)
    for i, (x,y) in enumerate(product(range(11), range(11))):
        current_axis = ax[x, y]
        cm = current_axis.imshow(images[i].reshape(28, 28))
        current_axis.set_title(labels[i], fontsize=5)

        current_axis.set_axis_off()
    fig.suptitle(title)
    plt.colorbar(cm, ax=ax)
    plt.show()

def preview_kernels(kernels, title):
    """
    Shows a few kernels in 2d
    """

    # We reshape kernels
    (w, h, a, b) = kernels.shape

    # We generate a plot
    (fig, ax) = plt.subplots(a, b)
    for x, y in product(range(a), range(b)):
        if a == 1:
            current_axis = ax[y]
        else:
            current_axis = ax[x, y]
        current_axis.imshow(kernels[:,:,x,y])
        current_axis.set_axis_off()
    fig.suptitle(title)
    plt.show()


def plot_learning_curves(train_loss, test_loss, title):
    """
    Plot learning curves
    """
    fig, ax = plt.subplots()
    ax.plot(train_loss, color="blue", label="Train-set")
    ax.plot(test_loss, color="green", label="Test-set")
    ax.set_ylabel("Loss")
    ax.set_xlabel("Epochs")
    ax.set_title(title)
    ax.legend()
    plt.show()

def show_image(im, title):
    """
    This function shows an image.
    """
    plt.imshow(im)
    plt.colorbar()
    plt.title(title)
    plt.show()

def show_binary(orig, binarized, title): 
    """
    This function shows the binarized version of the image
    """

    fig, ax = plt.subplots(2) 
    ax[0].imshow(orig)
    ax[0].set_title("Original")
    ax[1].imshow(binarized)
    ax[1].set_title("Binary")
    fig.suptitle(title)
    plt.show()


def preview_activations(model, sample, title):
    """
    This function allows to preview the activations through the model.
    """

    activations = backend.function([model.layers[0].input],
                                   [model.layers[0].output,
                                    model.layers[1].output,
                                    model.layers[2].output,
                                    model.layers[3].output,
                                    model.layers[4].output,
                                    model.layers[5].output,
                                    model.layers[6].output,
                                    model.layers[7].output])
    output = activations(sample)

    fig = plt.figure()
    fig.suptitle(title)
    plt.subplots_adjust(hspace=0.5)
    gs = gridspec.GridSpec(8, 1)

    def add_volume_activation(n, title):
        act = output[n]
        w = act.shape[-1]
        g = gridspec.GridSpecFromSubplotSpec(1, w, subplot_spec=gs[n])
        for i in range(w):
            ax = fig.add_subplot(g[i])
            ax.imshow(act[0,:,:,i])
            ax.set_xticks([])
            ax.set_yticks([])
            if float(w)/float(i+1) == 2:
                ax.set_title(title)

    add_volume_activation(0, "First Convolutional layer output")
    add_volume_activation(1, "First MaxPooling layer output")
    add_volume_activation(2, "Second Convolutional layer output")
    add_volume_activation(3, "Second MaxPooling layer output")

    def add_flat_activation(n, title):
        act = output[n]
        w = act.shape[-1]
        g = gridspec.GridSpecFromSubplotSpec(1, 1, subplot_spec=gs[n])
        ax = fig.add_subplot(g[0])
        ax.imshow(act)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(title)

    add_flat_activation(4, "Flatten layer output")
    add_flat_activation(5, "First Dense layer output")
    add_flat_activation(6, "Second Dense layer output")
    add_flat_activation(7, "Output layer output")

    plt.show()
