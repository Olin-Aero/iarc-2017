#ifndef __QROBOT_H__
#define __QROBOT_H__
#include <memory>

#include "robot.h"
#include "robot_item.h"

struct QRobot{
    public:
        std::shared_ptr<Robot> robot;
        RobotItem* item;
    public:
        QRobot(std::shared_ptr<Robot> r, RobotItem* i);
        void update(float dt);
};

#endif
