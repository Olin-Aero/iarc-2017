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
    def __init__(self):
        self.allocated_memory = .6
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
            angle (float): the angle relative to the drone
        """
        if len(img.shape) == 3:
            img = np.expand_dims(img, axis=0)
        y_pred = model.predict(x_test)
        angle = self.vec_to_angle(y_pred)
        return angle


    def limit_mem(self):
        """
        Limits the GPU memory allocated to this python process.
        We need to figure out how much memory to allocate when we get the 
        Nvidia tegra K1 as we still want to save memory for object detection
        """
        print "Limiting GPU memory to {} percent".format(self.allocated_memory)
        config = tf.ConfigProto()
        # config.gpu_options.allow_growth=True
        config.gpu_options.per_process_gpu_memory_fraction = self.allocated_memory
        sess = tf.Session(config=config)
        K.set_session(sess)


    def init_model(self):
        """ Initializes the neural network """
        print "Initializing Neural Network"
        self.model = load_model('mobilenet_finetune.hdf5',
                                custom_objects={'relu6': relu6,
                                'DepthwiseConv2D': DepthwiseConv2D})


if __name__ == '__main__':
    pass