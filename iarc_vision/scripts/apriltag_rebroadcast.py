#!/usr/bin/env python2
from PIL.ImageTransform import Transform

import rospy

from std_msgs.msg import Header
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PointStamped, Point, TransformStamped, \
    Transform, Quaternion
from tf import TransformBroadcaster, TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from iarc_main.msg import RoombaSighting, Roomba


class TagTransformer(object):
    """
    A TagTransformer is responsible for transforming detections by the apriltags_ros library
    into RoombaSighting messages for processing by the rest of the stack.
    """

    def __init__(self, linear_covariance=0.1, angular_covariance=0.5, tf_frame='odom', camera_fov=1.0):
        """
        :param linear_covariance: (meters) the linear covariance associated with a tag detection
        :param angular_covariance: (radians) the angular covariance of a tag
        :param tf_frame (str): The TF frame of the map / world.
        :param camera_fov (float): The FOV radius of the camera, in meters.
        """
        self.tf = TransformListener()
        self.pub = rospy.Publisher('visible_roombas', RoombaSighting, queue_size=0)

        self.tf_pub = TransformBroadcaster()

        cov = [0] * 36
        cov[0] = cov[7] = cov[14] = linear_covariance
        cov[21] = cov[28] = cov[35] = angular_covariance
        self.covariance = cov

        self.fov = camera_fov

        self.camera_frame = tf_frame  # TF frame is unknown until we get a message with a tag in it
        self.map_frame = tf_frame

        # Negates x and y coordinates of tag detections
        self.apply_apriltag_fix = True

        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.on_tags)

    def on_tags(self, msg):
        """
        Handler for AprilTag detection.
        Converts AprilTagDetectionArray to a RoombaSighting object, then publishes it.

        :type msg: AprilTagDetectionArray
        """
        sighting = RoombaSighting()

        for tag in msg.detections:  # type: AprilTagDetection

            if self.apply_apriltag_fix:
                tag.pose.pose.position.x *= -1
                tag.pose.pose.position.y *= -1

            roomba_frame_id = 'roombas/{}'.format(tag.id)

            # Transform the detection into map frame, and constrain it to be flat on the ground
            pose = self.tf.transformPose(self.map_frame, tag.pose)
            pose.pose.position.z = 0
            _, _, z_angle = euler_from_quaternion([getattr(pose.pose.orientation, s) for s in 'xyzw'])
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, z_angle))


            roomba = Roomba()
            roomba.visible_location = PoseWithCovarianceStamped(
                header=pose.header,
                pose=PoseWithCovariance(
                    pose=pose.pose,
                    covariance=self.covariance
                )
            )

            # Because April tags don't have color information, assign even tags to be GREEN and odd tags to be RED
            roomba.type = [Roomba.GREEN, Roomba.RED][tag.id % 2]

            roomba.last_seen = tag.pose.header.stamp

            roomba.frame_id = roomba_frame_id

            sighting.data.append(roomba)
            sighting.magical_ids.append(tag.id)

            self.camera_frame = tag.pose.header.frame_id

            # self.tf_pub.sendTransform(roomba.visible_location.pose.pose.position,
            #                           roomba.visible_location.pose.pose.orientation,
            #                           roomba.last_seen, roomba_frame_id, self.tf_frame)

            self.tf_pub.sendTransformMessage(
                TransformStamped(
                    header=roomba.visible_location.header,
                    child_frame_id=roomba.frame_id,
                    transform=Transform(
                        translation=roomba.visible_location.pose.pose.position,
                        rotation=roomba.visible_location.pose.pose.orientation
                    )
                )
            )

        # TODO: Use tf to get camera position and altitude at capture time for better FOV estimate
        # This potentially requires subscribing to tag_detections_poses as well, to get timestamps of empty
        # detection events.
        sighting.fov_center = PointStamped(
            header=Header(
                stamp=rospy.Time(0),
                frame_id=self.camera_frame
            ),
            point=Point()
        )
        sighting.fov_radius = self.fov

        self.pub.publish(sighting)


if __name__ == '__main__':
    rospy.init_node('tag_transformer')
    TagTransformer()
    rospy.spin()
