#!/usr/bin/env python

#import modules
import numpy as np
from PIL import Image
from scipy.ndimage import zoom
import rospy
import cv2
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image

from keras.models import Sequential
from keras.layers.convolutional import *
from keras.layers.core import Flatten, Dense
from keras.optimizers import Adam
from imageSubscriber import ImageSubscriber

from cmdVelPublisher import CmdVelPublisher
from imageSubscriber import ImageSubscriber

class RobotController(ImageSubscriber, object):
    def __init__(self):
        rospy.init_node('robot_controller')
        super(RobotController, self).__init__()

        # Load the neural network
        print "loading model"
        self.model = self.get_model()
        self.model.load_weights('epoche_2500.h5')


    def limit_mem(self):
        """
        Limits the GPU
        """

        print("limit_mem")
        config = tf.ConfigProto()
        # config.gpu_options.allow_growth=True
        config.gpu_options.per_process_gpu_memory_fraction = 0.6
        sess = tf.Session(config=config)
        K.set_session(sess)

    def get_model(self):
        """
        Creates the neural network
        """
        img_rows, img_cols = (64, 64)
        in_shape = (img_rows, img_cols, 3)
        model = Sequential([
            Convolution2D(32,3,3, border_mode='same', activation='relu', input_shape=in_shape),
            MaxPooling2D(),
            Convolution2D(64,3,3, border_mode='same', activation='relu'),
            MaxPooling2D(),
            Convolution2D(128,3,3, border_mode='same', activation='relu'),
            MaxPooling2D(),
            Flatten(),
            Dense(2048, activation='relu'),
            Dense(1024, activation='relu'),
            Dense(512, activation='relu'),
            Dense(1)
            ])
        model.compile(loss='mean_absolute_error', optimizer='adam')
        return model

    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                print self.angularVector
                time_start = rospy.get_time()
                cv2.imshow('video_window', self.cv_image)
                cv2.waitKey(5)
                self.angularVector = Vector3(z=self.model.predict(np.reshape(self.cv_image, (1,64,64,3)))[0][0])
                self.sendMessage()
                rospy.sleep(.49955 - (rospy.get_time() - time_start))
                print rospy.get_time() - time_start
            r.sleep()

if __name__ == '__main__':
    robot_controller = RobotController()
    robot_controller.run()
