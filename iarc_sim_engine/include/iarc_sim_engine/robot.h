#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string>

class Robot{
    protected:
        float x,y,z,t;
        float vx=0,vy=0,vz=0;
        float w=0;
        std::string name;
        Robot(const std::string name);
        virtual ~Robot();
    public:
        void reset();
        void update(float dt);

        void set_pos(float,float,float,float);
        void get_pos(float&,float&,float&,float&) const;

        void set_vel(float vx, float vy, float vz, float w);
        void get_vel(float&, float&, float&, float&) const;

        void set_name(const std::string name);
        void get_name(std::string& name) const;
};


#endif
