"""
This script loads an image, extracts sprites, and infer the classes with the
network trained earlier.
"""

#----------------------------------------------------------------------- IMPORTS

from src.detection import get_box_contours, get_sprites, segment_colors,\
    segment_thresholding
from keras.models import load_model
import imageio
import collections
import matplotlib.pyplot as plt
import glob

#----------------------------------------------------------------------- GLOBALS

LABELS=[1, 2]

#------------------------------------------------------------------------ STRUCT

Box = collections.namedtuple('Box', 'contour sprite label')

#----------------------------------------------------------------- PREPROCESSING

def preprocess(imag):
    """
    This image preprocesses the sprites from the detection pipeline to match
    the mnist data presented to the network.
    """
    ##################
    # YOUR CODE HERE #
    ##################


def preprocess_sprites(sprts, debug=False):
    """
    This function preprocesses sprites to make them closer to the mnist images.
    """

    out_sprites = []

    for imag in sprts:

        imag = preprocess(imag)

        if debug:
            plt.imshow(imag)
            plt.title("Pre-processed sprites")
            plt.colorbar()
            plt.show()

        out_sprites.append(imag)

    return out_sprites


def process(image, model, debug=None):
    """
    This function processes an image given a model, and returns a list of Box.
    """

    debug_inner = debug in ["inner", "all"]
    contours = get_box_contours(image, segment_colors, debug=debug_inner)
    sprites = get_sprites(image, contours, debug=debug_inner)
    inputs = preprocess_sprites(sprites, debug=debug_inner)
    labels = [model.predict(i.reshape(1, 28, 28, 1))\
              .squeeze().argmax() + 1 for i in inputs]

    boxes = [Box(contour=c, sprite=s, label=l) \
             for c, s, l in  zip(contours, sprites, labels)]

    if debug in ["all", "synthesis"]:
        for box in boxes:
            fig, ax = plt.subplots(nrows=2)
            ax[0].imshow(image)
            ax[0].plot(box.contour[:,0], box.contour[:,1], "og")
            ax[0].plot(box.contour.mean(axis=0)[0], \
                       box.contour.mean(axis=0)[1], "og")
            ax[1].imshow(box.sprite)
            ax[1].set_title("Label recognized: {}".format(box.label))
            plt.show()

    return boxes

#-------------------------------------------------------------------------- MAIN

if __name__ == "__main__":

    print("1) Load model")
    print("----------------")
    path = raw_input("Enter the path to your network file: ")
    model = load_model(path)
    print(model.summary)

    print("2) Process data")
    print("------------")
    test = glob.glob('data/cubes/ergo_cubes/**/*.jpg')
    for path in test:
        print("Testing image {}".format(path))
        image = imageio.imread(path)[:,:,0]
        try:
            boxes = process(image, model, debug="synthesis")
        except Exception as e:
            print("Failed to process image {}".format(path))
            pass
