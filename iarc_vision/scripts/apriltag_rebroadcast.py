#!/usr/bin/env python2

import rospy
from std_msgs.msg import Header
from iarc_main.msg import RoombaSighting, Roomba
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PointStamped, Point


class TagTransformer(object):
    def __init__(self, linear_covariance=0.1, angular_covariance=0.5):
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.on_tags)
        self.pub = rospy.Publisher('seen_roombas', RoombaSighting, queue_size=0)

        cov = [0] * 36
        cov[0] = cov[7] = cov[14] = linear_covariance
        cov[21] = cov[28] = cov[35] = angular_covariance
        self.covariance = cov

        self.tf_frame = 'map'  # TF frame is unknown until we get a message with a tag in it

    def on_tags(self, msg):
        """
        Handler for AprilTag detection
        :type msg: AprilTagDetectionArray
        """
        sighting = RoombaSighting()

        for tag in msg.detections:  # type: AprilTagDetection
            roomba = Roomba()
            roomba.visible_location = PoseWithCovarianceStamped(
                header=tag.pose.header,
                pose=PoseWithCovariance(
                    pose=tag.pose.pose,
                    covariance=self.covariance
                )
            )

            roomba.type = Roomba.GREEN

            roomba.last_seen = tag.pose.header.stamp

            sighting.data.append(roomba)
            sighting.magical_ids.append(tag.id)

            self.tf_frame = tag.pose.header.frame_id

        # TODO: Use tf to get camera position at capture time
        sighting.fov_center = PointStamped(
            header=Header(
                stamp=rospy.Time(0),
                frame_id=self.tf_frame
            ),
            point=Point()
        )
        sighting.fov_radius = 1.0

        self.pub.publish(sighting)


if __name__ == '__main__':
    rospy.init_node('tag_transformer')
    TagTransformer()
    rospy.spin()
