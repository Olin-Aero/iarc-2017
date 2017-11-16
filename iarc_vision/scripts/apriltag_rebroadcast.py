#!/usr/bin/env python2
from PIL.ImageTransform import Transform

import rospy

from std_msgs.msg import Header
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PointStamped, Point, TransformStamped, \
    Transform
from tf import TransformBroadcaster

from iarc_main.msg import RoombaSighting, Roomba


class TagTransformer(object):
    """
    A TagTransformer is responsible for transforming detections by the apriltags_ros library
    into RoombaSighting messages for processing by the rest of the stack.
    """

    def __init__(self, linear_covariance=0.1, angular_covariance=0.5, tf_frame='map', camera_fov=1.0):
        """
        :param linear_covariance: (meters) the linear covariance associated with a tag detection
        :param angular_covariance: (radians) the angular covariance of a tag
        :param tf_frame (str): The initial TF frame name to use before any detections come in.
        :param camera_fov (float): The FOV radius of the camera, in meters.
        """
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.on_tags)
        self.pub = rospy.Publisher('visible_roombas', RoombaSighting, queue_size=0)

        self.tf_pub = TransformBroadcaster()

        cov = [0] * 36
        cov[0] = cov[7] = cov[14] = linear_covariance
        cov[21] = cov[28] = cov[35] = angular_covariance
        self.covariance = cov

        self.tf_frame = tf_frame  # TF frame is unknown until we get a message with a tag in it
        self.fov = camera_fov

        # Negates x and y coordinates of tag detections
        self.apply_apriltag_fix = True

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

            roomba = Roomba()
            roomba.visible_location = PoseWithCovarianceStamped(
                header=tag.pose.header,
                pose=PoseWithCovariance(
                    pose=tag.pose.pose,
                    covariance=self.covariance
                )
            )

            roomba.type = [Roomba.GREEN, Roomba.RED][tag.id % 2]

            roomba.last_seen = tag.pose.header.stamp

            roomba.frame_id = roomba_frame_id

            sighting.data.append(roomba)
            sighting.magical_ids.append(tag.id)

            self.tf_frame = tag.pose.header.frame_id

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
                frame_id=self.tf_frame
            ),
            point=Point()
        )
        sighting.fov_radius = self.fov

        self.pub.publish(sighting)


if __name__ == '__main__':
    rospy.init_node('tag_transformer')
    TagTransformer()
    rospy.spin()
