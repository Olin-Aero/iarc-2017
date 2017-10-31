#include <gazebo/gazebo.hh>
#include <algorithm>
#include <functional>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/transport/transport.hh>

#include <map>
#include <iostream>

class Roomba
{
    protected:
        std::string left_joint;
        std::string right_joint;
        float wheel_radius;
        float wheel_separation;
        struct{
            std::string top_tap;
            std::string front_bumper;
            float lin_vel;
            float ang_vel;
        } params;
        // Pointer to the model
        physics::ModelPtr model;
        physics::JointPtr joint_l;
        physics::JointPtr joint_r;

        // Connection to Gazebo
        event::ConnectionPtr cxn;

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void setVelocity(float v, float w);
};
