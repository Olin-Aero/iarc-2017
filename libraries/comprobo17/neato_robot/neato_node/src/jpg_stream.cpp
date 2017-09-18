#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <netdb.h>
#include <netinet/in.h>
#include <curl/curl.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/asio/ip/tcp.hpp>
#include <sstream>

int main(int argc, char** argv)
{
  //boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  cv::VideoCapture cap;
  std::string hostname;

  ros::init(argc, argv, "jpg_stream");
  ros::NodeHandle nh("camera");
  image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
  image_transport::CameraPublisher cam_pub;
  camera_info_manager::CameraInfoManager cinfo_manager_(nh);
  cam_pub = it.advertiseCamera("image_raw", 10);

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  if (!ros::param::has("~host")) {
      ROS_ERROR("Need to set host");
      return -1;
  }
  ros::param::get("~host", hostname);
  std::ostringstream streamURL;

  streamURL << "http://" << hostname << ":11111/?action=stream&amp;type=.mjpg";

  cap.open(streamURL.str());

  //cv::namedWindow("myimage");
  while (nh.ok()) {
    cap >> frame; // get a new frame from camera
    if(!frame.empty()) {
      sensor_msgs::CameraInfo::Ptr cinfo(
        new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      msg->header.frame_id = "raspicam";
      cinfo->header.frame_id = "raspicam";
      cinfo->header.stamp = msg->header.stamp;

      cam_pub.publish(msg, cinfo);
    }
    //cv::imshow("myimage", frame);
    cv::waitKey(1);
    ros::spinOnce();
  }
}
