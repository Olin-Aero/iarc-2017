#!/usr/bin/env python

"""
Angle Predictor

Contact: Nathan Yee
Model weights/config: (figuring out how to do without saving on git)

"""

#import modules
import numpy as np
from PIL import Image
from keras.models import load_model
from keras.applications.mobilenet import relu6, DepthwiseConv2D
import keras.backend as K

class AnglePredictor(object):
    def __init__(self,
                 model_path='mobilenet_finetune.hdf5',
                 verbose=2,
                 allocated_mem=.6):
        """
        Initialize the angle predictor

        Args:
            model_path (string): path to the hdf5 file that contains
                                 the neural network config and weights
            verbose (int): 0 is no verbosity
                           1 is minimal verbosity
                           2 is full verbosity
            allocated_mem (float): the percent of GPU memory to allocate
                                   to this python process

        TODO:
            figure out how much memory to allocate given a Nvidia Tegra K1
        """
        # Assign default attributes
        self.model_path = model_path
        self.verbose = verbose
        self.allocated_mem = allocated_mem

        # Initialize AnglePredictor
        self.limit_mem()
        self.model = self.init_model()


    def vec_to_angle(self, vec):
        """ Turns a vector into its corresponding angle """
        return np.degrees(np.arctan2(vec[1], vec[0]))


    def predict_angle(self, img):
        """
        Predicts the angle of a roomba relative to the drone

        Args:
            img (ndarray): an image in RGB format

        Returns:
            angle (float): the angle from the top of the image
                           (counter clockwise positive, clockwise negative). 
                           A roomba at zero degrees looks like it is facing
                           directly up in the image.

        """
        if len(img.shape) == 3:
            img = np.expand_dims(img, axis=0)
        y_pred = model.predict(x_test)
        angle = self.vec_to_angle(y_pred)
        if self.verbose >= 2:
            print "Predicted angle is: {} degrees".format(angle)
        return angle


    def limit_mem(self):
        """
        Limits the GPU memory allocated to this python process.
        """
        if self.verbose >= 1:
            print "Limiting GPU memory to {} percent".format(self.allocated_mem)
        config = tf.ConfigProto()
        # config.gpu_options.allow_growth=True
        config.gpu_options.per_process_gpu_memory_fraction = self.allocated_mem
        sess = tf.Session(config=config)
        K.set_session(sess)


    def init_model(self):
        """ Initializes the neural network """
        if self.verbose >= 1:
            print "Initializing Neural Network"
        self.model = load_model(self.model_path,
                                custom_objects={'relu6': relu6,
                                'DepthwiseConv2D': DepthwiseConv2D})


if __name__ == '__main__':
    pass