#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string>

class Robot{
    protected:
        float x,y,t;
        float vx,vy,w;
        std::string name;
        Robot(const std::string name);
        virtual ~Robot();
    public:
        void reset(float x, float y, float t);
        void update(float dt);

        void set_pos(float,float,float);
        void get_pos(float&,float&,float) const;

        void set_vel(float vx, float vy, float w);
        void get_vel(float,float,float) const;

        void set_name(const std::string& name);
        void get_name(std::string& name) const;
};


#endif
