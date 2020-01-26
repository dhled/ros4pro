"""
This module contains the functions used for the preprocessing phase of the recognition.
"""
import imageio
import glob
import matplotlib.pyplot as plt
import numpy as np
import skimage.transform as tf
from skimage.filters import threshold_otsu
from skimage.measure import (approximate_polygon, find_contours)
from skimage.morphology import (closing, convex_hull_object, square)
from skimage.segmentation import clear_border
from scipy.spatial import ConvexHull

import sys
sys.path.append(".")
import vis


def normalize(imag, mean, std, debug=False):
    """
    This function returns a normalized copy of the image.
    """

    imag = imag.copy()

    imag = imag - mean
    imag = imag / float(std)

    if debug:
        plt.imshow(imag)
        plt.title("normalized image")
        plt.show()

    return imag


def binarize(imag, debug=False):
    """
    This function returns a binarized version of the image.
    """

    imag = imag.copy()

    thresh = threshold_otsu(imag)
    imag = imag > thresh
    imag = closing(imag, square(3))
    imag = clear_border(imag)
    imag = convex_hull_object(imag)

    if debug:
        fig, ax = plt.subplots()
        ax.imshow(imag)
        fig.suptitle("Binary image")
        plt.show()

    return imag


def inverse(imag, debug=False):
    """
    This function returns an inversed image.
    """

    imag = imag.copy()

    imag = 1. - imag

    if debug:
        plt.imshow(imag)
        plt.title("Thresholded image")
        plt.show()

    return imag


def approximate_square(contour):
    """
    This function approximates a contour with a square.
    """

    tol = 50

    # While the right number of segments is not found, we modify the tolerance consequently.
    for _ in range(50):
        coords = approximate_polygon(contour, tolerance=tol)
        coords = np.flip(coords[0:-1], axis=1)
        if coords.shape[0] == 4:
            return coords
        if coords.shape[0] < 4:
            print("Failed to approximate square with tolerance {}, found {} points. Retrying."\
                .format(tol, coords.shape[0]))
            tol -= 1
        else:
            print("Failed to approximate square with tolerance {}, found {} points. Retrying."\
                .format(tol, coords.shape[0]))
            tol += 1

    raise Exception("Failed to approximate square")


def reorder_contour(contour):
    """
    This function allows to reorder the contour so that the down-right point is always first, and
    points are ordered clockwise.
    """

    # We reorder the points
    rightest_points = contour[contour[:, 0].argsort()][-2:]
    lowest_rightest_point = rightest_points[rightest_points[:, 1].argsort()][-1]
    lr_point_idx = np.where(np.all(contour == lowest_rightest_point, axis=1))[0][0]
    contour = np.roll(contour, -lr_point_idx, axis=0)

    return contour


def get_box_contours(imag, debug=False):
    """
    This function takes as input a single channel image, and returns a list of contours bounding
    the cubes.
    """

    # We make sure that we work on a local copy of the image
    imag = imag.copy()

    binar = binarize(imag)
    ctrs = find_contours(binar, 0)
    hulls = [ConvexHull(c) for c in ctrs]
    ctrs = [h.points[np.flip(h.vertices)] for h in hulls if h.area > 460]
    ctrs = [approximate_square(c) for c in ctrs]
    ctrs = [reorder_contour(c) for c in ctrs]

    if debug:
        plt.imshow(imag)
        plt.imshow(binar, alpha=0.4)
        for coords in ctrs:
            plt.plot(coords[:, 0], coords[:, 1], 'og', linewidth=2)
            plt.plot(coords.mean(axis=0)[0], coords.mean(axis=0)[1], 'or')
            ind = [1, 2, 3, 4]
            for i, txt in enumerate(ind):
                plt.annotate(txt, (coords[i, 0], coords[i, 1]))
        plt.title("Contours found")
        plt.show()

    return ctrs


def get_sprites(imag, ctrs, debug=False):
    """
    This function computes a projective transform from the source (mnist image) to
    the destination (contour) and extracts the warped sprite.
    """

    # We make sure that we work on a local copy of the image
    imag = imag.copy()

    # We loop through the sprites
    sprts = []

    for contour in ctrs:

        # We compute the projective transform
        source_points = np.array([[28, 28], [0, 28], [0, 0], [28, 0]])
        destination_points = contour
        transform = tf.ProjectiveTransform()
        transform.estimate(source_points, destination_points)

        # We transform the image
        warped = tf.warp(imag, transform, output_shape=(28, 28))

        if debug:
            _, axis = plt.subplots(nrows=2, figsize=(8, 3))
            axis[0].imshow(imag)
            axis[0].plot(destination_points[:, 0], destination_points[:, 1], '.r')
            axis[1].imshow(warped)
            plt.show()

        sprts.append(warped)

    return sprts


def preprocess_sprites(sprts, debug=False):
    """
    This function preprocesses sprites to make them closer to the mnist images.
    """

    out_sprites = []

    for imag in sprts:

        # We make a local copy
        imag = imag.copy()

        ##################
        # YOUR CODE HERE #
        ##################

        if debug:
            plt.imshow(imag)
            plt.title("Pre-processed sprites")
            plt.colorbar()
            plt.show()

        out_sprites.append(imag)

    return out_sprites


if __name__ == "__main__":

    print("1) Loading images:")
    print("------------------")
    test_data = glob.glob('../data/cubes/*/*.jpg')
    print("Found test images: {}".format(test_data))
    images = []
    for path in test_data:
        images.append(imageio.imread(path)[:, :, 0])
    images = np.array(images)
    vis.show_image(images[0], "Image sample")
    raw_input("Answer questions in 3.1 and press enter to continue... ")

    print("2) Binarizing image")
    print("-------------------")
    for im in images:
        binarize(im, debug=True)
    raw_input("Answer questions in 3.2 and press enter to continue...")

    print("3) Getting boxes")
    print("----------------")
    ctrs = []
    for im in images:
        ctrs.append(get_box_contours(im, debug=True))
    raw_input("Answer questions in 3.3 and press enter to continue...")

    print("4) Getting sprites")
    print("------------------")
    sprites = []
    for i in range(images.shape[0]):
        sprites.append(get_sprites(images[i], ctrs[i], debug=True))
    raw_input("Answer questions in 3.4 and press enter to continue...")

    print("5) Pre-processing")
    print("-----------------")
    for sprt in sprites:
        preprocess_sprites(sprt, debug=True)
    raw_input("Answer questions in 3.5 and press enter to continue...")
