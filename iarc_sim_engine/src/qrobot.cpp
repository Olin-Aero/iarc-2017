#include "qrobot.h"

QRobot::QRobot(
        std::shared_ptr<Robot> r,
        RobotItem* i
        ):robot(r), item(i){
}

void QRobot::render(){
    float x,y,t;
    robot.get_pos(x,y,t);
    QPointF pos(x,y);
    item->set_pos(pos,t);
}
