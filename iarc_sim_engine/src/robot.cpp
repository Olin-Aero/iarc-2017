#include "robot.h"
#include <cmath>

Robot::Robot(const std::string name):name(name){}
Robot::~Robot(){}

void Robot::reset(float _x, float _y, float _t){
    x=_x;
    y=_y;
    t=_t;
}
void Robot::update(float dt){
    //t w.r.t top
    x += v*cos(t)*dt;
    y += v*sin(t)*dt;

    // strafing
    x += -vy*sin(t)*dt;
    y += vy*cos(t)*dt;

    t += w*dt;
}
void Robot::set_pos(float _x, float _y, float _t){
    reset(_x,_y,_t);
}
void Robot::get_pos(float& _x, float& _y, float& _t) const{
    _x=x;
    _y=y;
    _t=t;
}
void Robot::set_vel(float _v, float _w, float _vy){
    v=_v;
    w=_w;
    vy=_vy;
}
void Robot::get_vel(float& _v, float& _w) const{
    _v=v;
    _w=w;
}
void Robot::set_name(const std::string _name){
    name=_name;
}
void Robot::get_name(std::string &_name) const{
    _name=name;
}

//class Roomba : public Robot{
//
//};
//
//class Drone : public Robot{
//
//};
