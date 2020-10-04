"""
This module contains the functions used for the preprocessing phase of the
recognition.
"""

#----------------------------------------------------------------------- IMPORTS

import imageio
import glob
import matplotlib.pyplot as plt
import numpy as np
import skimage.transform as tf
from skimage.filters import threshold_minimum, threshold_otsu
from skimage.measure import (approximate_polygon, find_contours,
                             moments_coords, perimeter)
from skimage.morphology import (opening,closing, convex_hull_object, square)
from skimage.segmentation import clear_border
from skimage.exposure import histogram
from scipy.spatial import ConvexHull
from skimage import data, io, segmentation, color
from skimage.future import graph
from scipy import ndimage

import sys
sys.path.append(".")
import vis

#----------------------------------------------------------------------- HELPERS

def show_shape(imag, shape, title):
    """
    Helper function to plot an image, with a shape as overlay.
    """
    plt.imshow(imag)
    plt.imshow(shape, alpha=0.3)
    plt.title(title)
    plt.show()


def show_image(imag, title):
    """
    Helper function to plot an image.
    """
    plt.imshow(imag)
    plt.title(title)
    plt.title("{}\nShape: {} Dtype: {} Min: {} Max: {}"\
              .format(title,\
                      get_image_shape(imag),\
                      get_image_dtype(imag),\
                      get_image_min(imag),\
                      get_image_max(imag)))
    plt.show()


def get_image_min(imag):
    """
    This function returns the minimum value of the image.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def get_image_max(imag):
    """
    This function returns the maximum value of the image.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def get_image_shape(imag):
    """
    This function returns the shape of the image.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def get_image_dtype(imag):
    """
    This function returns the data type of the image.
    """
    ##################
    # YOUR CODE HERE #
    ##################


#---------------------------------------------------------THRESHOLD SEGMENTATION

def turn_to_grey(imag):
    """
    This function returns an image corresponding to the grey level flattening
    of the input image
    """
    ##################
    # YOUR CODE HERE #
    ##################


def compute_image_threshold(imag):
    """
    This function computes the threshold value for the input image.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def build_binary_image(imag, thresh):
    """
    This function computes a binary image where the pixels in the `imag`
    superior to the threshold become ones, and the other become zeros.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def perform_closing(imag):
    """
    This function performs a closing on the input image.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def perform_opening(imag):
    """
    This function performs a closing on the input image.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def clean_shape(imag):
    """
    This function cleans a separated shape so that it only contains convex
    white areas representing the cubes.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def segment_thresholding(imag, debug=False):
    """
    This function extracts the location of the cubes based on their light
    level. This function returns a list of binary images, each containing the
    mask of one cube.
    """

    imag = imag.copy()

    # We turn the image to a grey level image
    imag = turn_to_grey(imag)
    if debug: show_image(imag, "Grey level image")

    # We compute the threshold and form the binary image
    thresh = compute_image_threshold(imag)
    shape = build_binary_image(imag, thresh)
    if debug: show_image(shape, "Raw binary image")

    # We clear some segmentation artifacts with morphological operation
    shape = perform_closing(shape)
    shape = perform_opening(shape)
    if debug: show_image(shape, "Binary image after closing")

    # We separate the different contiguous parts of the binary image.
    shapes, n_shapes = ndimage.label(shape)
    if debug: show_image(shapes, "Shapes detected")
    cubes = []
    for i in range(1, int(n_shapes+1)):
        shape = shapes==i
        shape = clean_shape(shape)
        cubes.append(shape)
        if debug: show_shape(imag, shape, "Cube detected")

    return cubes

#----------------------------------------------------- COLORIMETRIC SEGMENTATION

def build_segmented_image(imag):
    """
    This function takes an image as input, and turns it to a segmented image,
    which pixels are integer between 0 and n_classes, representing the
    different classes of objects segmented.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def clean_shape(imag):
    """
    This function allows to clean the raw shapes found by the segmentation.
    This function should transform the shape so that it is easy to tell from
    the shape whether it was a cube or not.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def is_cube(imag, mask):
    """
    This function evaluates if the shape of the input binary image is a cube
    or not.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def segment_colors(imag, debug=False):
    """
    This function extracts the location of the cubes based on their color
    level. A segmentation is performed on the whole image using the felzenszwalb
    method. The segmented shapes are cleaned, and their shape is asserted to be
    a square. This function returns a list of binary images, each containing the
    mask of one cube.
    """

    imag = imag.copy()

    # We perform the segmentation of the image.
    labels = build_segmented_image(imag)
    if debug: show_image(labels, "Felzenszwalb segmentation")

    # We evaluate each segmented shape.
    output = []
    for i in range(labels.max()+1):
        # We extract the mask of the current shape
        shape = labels==i
        # We clean the shape
        shape = clean_shape(shape)
        # We check if the shape is a cube or not
        if is_cube(imag, shape):
            if debug: show_shape(imag, shape, "Cube detected")
            output.append(shape)

    return output

#------------------------------------------------------------ CONTOUR EXTRACTION

def extract_raw_contour(shape):
    """
    This function returns a numpy array of shape (n, 2), where n is a variable
    number of points along the contour, each represented by two dimensions. The
    first dimension must be the x position of the point, and the second must be
    the y position of the point.
    If no contour was found, the function must return None.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def simplify_contour(image, contour):
    """
    This function simplifies a raw contour, so that it only keeps the four
    corners of the contour. The order of the points must be clockwise, with the
    first being the lower-right one.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def get_box_contours(imag, segment, debug=False):
    """
    This function takes as input an image, and returns a list of contours
    bounding the cubes. The argument `segment` is a function used to segment
    the image prior to extracting contours.
    """

    # We extract the shapes using the segmentation function.
    shapes = segment(imag, debug=True)

    # We process each shape from the segmentation
    output = []
    for shape in shapes:

        # We extract the shape (there is only one contour on the image)
        contour = extract_raw_contour(shape)

        # If no contour is found, we skip to the next shape
        if contour is None: continue

        # We simplify the contour
        contour = simplify_contour(shape, contour)

        output.append(contour)

    if debug:
        plt.imshow(imag)
        for coords in output:
            plt.plot(coords[:, 0], coords[:, 1], 'og', linewidth=2)
            plt.plot(coords.mean(axis=0)[0], coords.mean(axis=0)[1], 'or')
            ind = list(range(coords.shape[0]))
            for i, txt in enumerate(ind):
                plt.annotate(txt, (coords[i, 0], coords[i, 1]))
        plt.title("Contours found")
        plt.show()

    return output

#----------------------------------------------------------------------- SPRITES

def compute_transform(contour):
    """
    This function compute the transform between the mnist image plane, and the
    contour of the cube we want to extract.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def transform_image(imag, transform):
    """
    This function transform the image.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def get_sprites(imag, ctrs, debug=False):
    """
    This function computes a projective transform from the source (mnist image)
    to the destination (contour) and extracts the warped sprite.
    """

    # We make sure that we work on a local copy of the image
    imag = imag.copy()

    # We loop through the sprites
    sprts = []

    for contour in ctrs:

        # We compute the projective transform
        transform = compute_transform(contour)

        # We transform the image
        warped = transform_image(imag, transform)

        if debug:
            _, axis = plt.subplots(nrows=2, figsize=(8, 3))
            axis[0].imshow(imag)
            axis[1].imshow(warped)
            plt.show()

        sprts.append(warped)

    return sprts

#-------------------------------------------------------------------------- MAIN

if __name__ == "__main__":

    print("0) Loading images:")
    print("------------------")
    test_data = glob.glob('../data/ergo_cubes/dark/*.png')
    print("Found test images: {}".format(test_data))
    images = []
    for path in test_data:
        images.append(imageio.imread(path)[:, :])
    images = np.array(images)

    print("1) Showing samples:")
    print("------------------")
    for im in images:
        show_image(im, "Image sample")
    raw_input("Answer questions in 1.1 and press enter to continue... ")

    print("2) Segmenting image")
    print("-------------------")
    for im in images:
        im = segment_thresholding(im, debug=True)
    raw_input("Answer questions in 1.2 and press enter to continue...")

    print("3) Getting boxes")
    print("----------------")
    ctrs = []
    for i, im in enumerate(images):
        ctrs.append(get_box_contours(im, segment_thresholding, debug=True))
    raw_input("Answer questions in 3.3 and press enter to continue...")

    print("4) Getting sprites")
    print("------------------")
    sprites = []
    for i in range(images.shape[0]):
        sprites.append(get_sprites(images[i], ctrs[i], debug=True))
    raw_input("Answer questions in 1.4 and press enter to continue...")
