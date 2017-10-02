#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "robot_item.h"

class Robot{
    protected:
        float x,y,t;
        float vx,vy,w;
        std::string name;
        Robot(){}
        virtual ~Robot(){}
    public:
        void reset(float x, float y, float t);
        void render();

        void set_pos(float,float,float);
        void get_pos(float&,float&,float);

        void set_vel(float vx, float vy, float w);
        void get_vel(float,float,float);
};


#endif
