#include "qrobot.h"

QRobot::QRobot(
        std::shared_ptr<Robot> r,
        RobotItem* i
        ):robot(r), item(i){
}

void QRobot::update(float dt){
    float x,y,z,t;
    robot->update(dt);
    robot->get_pos(x,y,z,t);
    QPointF pos(x,y);
    item->set_pos(pos,t);
}
