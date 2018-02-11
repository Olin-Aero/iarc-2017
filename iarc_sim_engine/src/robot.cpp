#include "robot.h"
#include <cmath>

Robot::Robot(const std::string name):name(name){}
Robot::~Robot(){}

void Robot::reset(){
    set_pos(0,0,0,0);
}

void Robot::update(float dt){
    //t w.r.t top
    x += vx*cos(t)*dt;
    y += vx*sin(t)*dt;

    // strafing
    x += -vy*sin(t)*dt;
    y += vy*cos(t)*dt;

    z += vz*dt;

    if(z < 0){
        z=0;
    }

    t += w*dt;
}
void Robot::set_pos(float _x, float _y, float _z, float _t){
    x=_x;
    y=_y;
    z=_z;
    t=_t;
}
void Robot::get_pos(float& _x, float& _y, float& _z, float& _t) const{
    _x=x;
    _y=y;
    _z=z;
    _t=t;
}
void Robot::set_vel(float _vx, float _vy, float _vz, float _w){
    vx=_vx;
    vy=_vy;
    vz=_vz;
    w=_w;
}
void Robot::get_vel(float& _vx, float& _vy, float& _vz, float& _w) const{
    _vx=vx;
    _vy=vy;
    _vz=vz;
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
