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

// durations
#define T_INIT (0.00F)
#define T_DELAY (0.015F)

namespace gazebo
{
    class ObstacleRoomba : public ModelPlugin
    {
        enum State{WAIT,RUN,STOP,END};
        struct{
            std::string left_joint;
            std::string right_joint;
            std::string front_bumper;
            std::string pole_link;

            float wheel_radius;
            float wheel_separation;
            float lin_vel;
            float ang_vel;
            float bound;
        } params;

        private:
        // Pointer to the model
        physics::ModelPtr model;
        physics::JointPtr joint_l;
        physics::JointPtr joint_r;

        // Connection to Gazebo
        event::ConnectionPtr cxn;
        transport::NodePtr node;
        transport::SubscriberPtr contactSub;

        // Data
        State state;
        float t_trans; // how long to turn for
        bool col_flag;

        public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;
            
            for(auto& tag : {"front_bumper", "wheel_separation", "wheel_radius", "pole_link", "left_joint", "right_joint", "lin_vel", "ang_vel", "bound"}){
                if(!_sdf->HasElement(tag)){
                    gzerr << "SDF Element <" << tag << "> undefined" << std::endl;
                }
            }

            params.left_joint = _sdf->Get<std::string>("left_joint");
            params.right_joint = _sdf->Get<std::string>("right_joint");
            params.front_bumper = _sdf->Get<std::string>("front_bumper");
            params.pole_link = _sdf->Get<std::string>("pole_link");

            params.wheel_radius = _sdf->Get<float>("wheel_radius");
            params.wheel_separation = _sdf->Get<float>("wheel_separation");
            params.lin_vel = _sdf->Get<float>("lin_vel");
            params.ang_vel = _sdf->Get<float>("ang_vel");
            params.bound = _sdf->Get<float>("bound");

            this->joint_l = model->GetJoint(params.left_joint);
            this->joint_r = model->GetJoint(params.right_joint);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->cxn = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ObstacleRoomba::OnUpdate, this, std::placeholders::_1));

            state = WAIT;
            t_trans = T_INIT;

            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->GetName());

            std::vector<std::string> cols;
            for(auto& l : model->GetLinks()){
                for(auto& c : l->GetCollisions()){
                    auto s = c->GetScopedName();
                    if (s.find(params.pole_link) != std::string::npos ||
                            s.find(params.front_bumper) != std::string::npos){
                        //filter s
                        cols.push_back(s);
                    }
                }
            }

            auto cmgr = model->GetWorld()->GetPhysicsEngine()->GetContactManager();
            std::string topic = cmgr->CreateFilter(model->GetName(),cols);
            this->contactSub = this->node->Subscribe(topic, &ObstacleRoomba::OnContact, this);
        }

        State wait(float now){
            if(now > t_trans)
                return RUN;
            return WAIT;
        }

        State run(float now){
            setVelocity(params.lin_vel, -params.ang_vel);
            return RUN;
        }

        State stop(float now){
            setVelocity(0,0);
            if(now > t_trans){
                return RUN;
            }
            return STOP;
        }

        State end(float){
            setVelocity(0,0);
            return END;
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

        void OnContact(ConstContactsPtr& _msg){
            for(int i=0; i<_msg->contact_size();++i){
                if(_msg->contact(i).has_collision1()){
                    std::string c = _msg->contact(i).collision1();
                    col_flag |= c.find(params.front_bumper) != std::string::npos;
                    col_flag |= c.find(params.pole_link) != std::string::npos;
                }

                if(_msg->contact(i).has_collision2()){
                    std::string c = _msg->contact(i).collision2();
                    col_flag |= c.find(params.front_bumper) != std::string::npos;
                    col_flag |= c.find(params.pole_link) != std::string::npos;
                }
            }
        }

        // Called by the world update start event
        void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            if(state==END)
                return;

            auto now = this->model->GetWorld()->GetSimTime().Float();
            // check and clear flags
            if(col_flag){
                col_flag = false;
                t_trans = now + T_DELAY;
                state = STOP;
            }

            // run FSM
            State s_nxt;
            switch(state){
                case WAIT:
                    s_nxt=wait(now);
                    break;
                case RUN:
                    s_nxt=run(now);
                    break;
                case STOP:
                    s_nxt=stop(now);
                    break;
                case END:
                    s_nxt=end(now);
                    break;
            }
            state = s_nxt;

            auto p = model->GetWorldPose().pos;
            if(fabs(p.x) > params.bound || fabs(p.y) > params.bound)
                state=END;
        }
    };
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ObstacleRoomba)
}
