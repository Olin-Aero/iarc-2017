#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <random>


class NoisyOdometryNode{
	bool _initialized;
	float _rate;
	float _noise_lin;
	float _noise_ang;
	ros::Time _t0;

	std::string _odom_in;
	std::string _odom_out;

	ros::NodeHandle _nh;
	ros::Subscriber _sub;
	ros::Publisher  _pub;
	nav_msgs::Odometry odom_msg;
	
	std::default_random_engine _gen;
	std::normal_distribution<float> _dist_l, _dist_a;

	public:
	NoisyOdometryNode(){
		ros::param::get("~odom_in", _odom_in);
		ros::param::get("~odom_out", _odom_out);
		ros::param::get("~rate", _rate);
		ros::param::get("~noise_lin", _noise_lin);
		ros::param::get("~noise_ang", _noise_ang);

		_dist_l = std::normal_distribution<float>(0.0, _noise_lin);
		_dist_a = std::normal_distribution<float>(0.0, _noise_ang);

		_sub = _nh.subscribe(_odom_in, 10, &NoisyOdometryNode::odom_cb, this);
		_pub = _nh.advertise<nav_msgs::Odometry>(_odom_out, 10);
	}

	float zzl(float x){
		return x + _dist_l(_gen);
	}
	float zza(float x){
		return x + _dist_a(_gen);
	}

	void odom_cb(const nav_msgs::OdometryConstPtr& msg){
		if(!_initialized){
			odom_msg = *msg;
			_initialized = true;
			_t0 = ros::Time::now();
		}else{
			const geometry_msgs::Vector3& a = msg->twist.twist.angular;
			const geometry_msgs::Vector3& l = msg->twist.twist.linear;

			// manage time ...
			const ros::Time& t1 = msg->header.stamp;
			float dt = (t1 - _t0).toSec();
			if(!(dt>=0)) return;
			_t0 = t1;

			// apply linear vel
			auto& p = odom_msg.pose.pose.position;
			p.x += zzl(l.x)*dt;
			p.y += zzl(l.y)*dt;
			p.z += zzl(l.z)*dt;

			// apply angular twist
			tf::Quaternion q, dq;
			tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q);
			dq.setRPY(zza(a.x)*dt, zza(a.y)*dt, zza(a.z)*dt);
			tf::quaternionTFToMsg(q*dq, odom_msg.pose.pose.orientation);

			// copy data
			odom_msg.child_frame_id = msg->child_frame_id;
			odom_msg.header = msg->header;
			odom_msg.header.frame_id = "odom";
			odom_msg.twist = msg->twist;
		}
		_pub.publish(odom_msg);
	}

	void run(){
		ros::Rate rate(_rate);
		while(ros::ok()){
			ros::spinOnce();
			rate.sleep();
		}
	}
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "noisy_odometry");
	NoisyOdometryNode app;
	app.run();
}
