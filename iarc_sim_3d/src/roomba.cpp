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
        struct{
            std::string left_joint;
            std::string right_joint;
            float wheel_radius;
            float wheel_separation;
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
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;

            for(auto& tag : {"front_bumper", "top_tap", "wheel_separation", "wheel_radius", "left_joint", "right_joint", "lin_vel", "ang_vel"}){
                if(!_sdf->HasElement(tag)){
                    gzerr << "SDF Element <" << tag << "> undefined" << std::endl;
                }
            }

            params.left_joint = _sdf->Get<std::string>("left_joint");
            params.right_joint = _sdf->Get<std::string>("right_joint");
            params.wheel_radius = _sdf->Get<float>("wheel_radius");
            params.wheel_separation = _sdf->Get<float>("wheel_separation");
            params.top_tap = _sdf->Get<std::string>("top_tap");
            params.front_bumper = _sdf->Get<std::string>("front_bumper");
            params.lin_vel = _sdf->Get<float>("lin_vel");
            params.ang_vel = _sdf->Get<float>("ang_vel");

            this->joint_l = model->GetJoint(params.left_joint);
            this->joint_r = model->GetJoint(params.right_joint);
        }

        void setVelocity(float v, float w){
            auto l = params.wheel_separation;
            auto wr = params.wheel_radius;

            auto vw_r = v + w*l/2.0;
            auto vw_l = v - w*l/2.0;
            auto w_r = vw_r / wr;
            auto w_l = vw_l / wr;

            if(joint_l && joint_r){
                joint_l->SetVelocity(0, w_l);
                joint_r->SetVelocity(0, w_r);
            }
        }
};
