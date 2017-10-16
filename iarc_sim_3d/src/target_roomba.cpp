#include <gazebo/gazebo.hh>
#include <algorithm>
#include <functional>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Exception.hh>
#include <stdio.h>

// durations
#define T_COL (2.15F)
#define T_TOPTAP (0.5375F)
#define T_NOISE (0.85F)

// intervals
#define INT_NOISE (5.0F)
#define INT_REVERSE (20.0F)

#define LIN_VEL (0.33F)
#define WHEEL_SEP (0.34F)
#define ANG_VEL (LIN_VEL/(WHEEL_SEP/2.0))

namespace gazebo
{
    class TargetRoomba : public ModelPlugin
    {
        enum State{WAIT,RUN,TURN};
        private:
        // Pointer to the model
        physics::ModelPtr model;
        // Connection to Gazebo
        event::ConnectionPtr cxn;

        // Data
        State state;
        float s_begin; // time since state
        float t_turn; // how long to turn for

        float prv; //previous call-time
        public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->cxn = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ModelPush::OnUpdate, this, _1));
            this->prv = model->GetWorld()->GetSimTime().Float();

            if(_sdf->HasElement("collision")){
                _sdf->G
            }else{
                gzthrow("[Target Roomba] SDF Must Specify Element <collision>");
            }
            state = WAIT;
            //_sdf->HasElement("...")
        }

        State wait(float){
            return RUN;
        }
        State run(float now){
            this->model->SetLinearVel(math::Vector3(LIN_VEL,0,0));
            this->model->SetAngularVel(math::Vector3(0,0,0));

            /*
            if(fmod(prv, INT_NOISE) > fmod(now, INT_NOISE)){ // every 5-sec
                //addNoiseToVel();
                auto p = model->GetWorldPose;
                auto yaw = math::Rand::GetDblUniform(0.0, 20.0);
                Quaternion(0,0,yaw);
                p.CoordRotationAdd(r);
                model->SetWorldPose(p);
            }

            if(fmod(prv, INT_REVERSE) > fmod(now, INT_REVERSE)){ //every 20-sec
                //Reverse();

            }
            */
        }

        State turn(float now){
            this->model->SetLinearVel(math::Vector3(0,0,0));
            this->model->SetAngularVel(math::Vector3(0,0,-ANG_VEL)); //clockwise
            if(now > s_begin + t_turn){
                return RUN;
            }else{
                return TURN;
            }
        }

        // Called by the world update start event
        void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            auto now = this->model->GetWorld()->GetSimTime().Float();
            State s_nxt;
            switch(state){
                case WAIT:
                    s_nxt=wait(now);
                    break;
                case RUN:
                    s_nxt=run(now);
                    break;
                case TURN:
                    s_nxt=turn(now);
                    break;
            }

            if(s_nxt != state){
                s_begin = now;
            }

            this->prv = now;
            this->model->SetLinearVel(math::Vector3(.33, 0, 0)); //always go forward!
        }
    };
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
