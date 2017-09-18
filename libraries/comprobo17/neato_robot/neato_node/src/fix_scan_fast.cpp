#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <math.h>

laser_geometry::LaserProjection* projector_ = 0;
tf::TransformListener* listener_ = 0;
sensor_msgs::PointCloud* prev_cloud = 0;
ros::Publisher pub;
ros::Publisher pub_cloud;



void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  std::cout << "got a message!!!" <<std::endl;
  if (listener_ == 0 || projector_ == 0) {
    return;
  }
  std::cout << "GOT A SCAN!!!" << std::endl;
  sensor_msgs::LaserScan scan_mod(*scan_in);
  scan_mod.time_increment = (float)1.0/(360*5);
  scan_mod.scan_time = (float)1.0/5;
  if(!listener_->waitForTransform(
        scan_in->header.frame_id,
        "odom",
        scan_in->header.stamp + ros::Duration().fromSec(scan_mod.ranges.size()*scan_mod.time_increment),
        ros::Duration(1.0))){
     return;
  }

  double t_start = ros::Time::now().toSec();
  int best_matching_offset = -1;
  float best_match_val = -1.0;
  for (int t_offset_msecs = 200; t_offset_msecs <= 400; t_offset_msecs += 10) {
    float match_val = 0.0;
    sensor_msgs::PointCloud cloud;
    scan_mod.header.stamp = scan_in->header.stamp - ros::Duration(t_offset_msecs/1000.0);
    try {
      projector_->transformLaserScanToPointCloud("/odom",scan_mod,
            cloud,*listener_);
      if (prev_cloud == 0) {
        prev_cloud = new sensor_msgs::PointCloud(cloud);
        return;
      }
      for (int i = 0; i < cloud.points.size(); i++) {
        float min_val = -1.0;
        float dist;
        for (int j = 0; j < prev_cloud->points.size(); j++){
          dist = sqrt((prev_cloud->points[j].x - cloud.points[i].x)*(prev_cloud->points[j].x - cloud.points[i].x) +
                 (prev_cloud->points[j].y - cloud.points[i].y)*(prev_cloud->points[j].y - cloud.points[i].y) +
                 (prev_cloud->points[j].z - cloud.points[i].z)*(prev_cloud->points[j].z - cloud.points[i].z));
          if (min_val == -1 || dist < min_val) {
            min_val = dist;
          }
        }
        match_val += min_val;
      }
      if (best_matching_offset == -1 || match_val < best_match_val) {
        best_match_val = match_val;
        best_matching_offset = t_offset_msecs;
      }
    } catch (...) {
    }
  }
  // set this based on the best matching offset
  if (best_matching_offset < 0) {
    best_matching_offset = 0;
  }
  scan_mod.header.stamp = scan_in->header.stamp - ros::Duration(best_matching_offset/1000.0);
  std::cout << "PROCESSED A SCAN!!! " << best_matching_offset << " " << ros::Time::now().toSec() - t_start << std::endl;
  // Do something with cloud.
  pub.publish(scan_mod);
  try {
    sensor_msgs::PointCloud final_cloud;
    projector_->transformLaserScanToPointCloud("/odom",scan_mod,
              final_cloud,*listener_);
    pub_cloud.publish(final_cloud);
    delete prev_cloud;
    prev_cloud = new sensor_msgs::PointCloud(final_cloud);
  } catch (...) {}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "fix_scan_fast");

  listener_ = new tf::TransformListener();
  projector_ = new laser_geometry::LaserProjection();
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/scan",10,scanCallback);
  pub = node.advertise<sensor_msgs::LaserScan>("/stable_scan",10);
  pub_cloud = node.advertise<sensor_msgs::PointCloud>("/projected_stable_scan",10);

  ros::Rate rate(10.0);
  while (node.ok()){
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};