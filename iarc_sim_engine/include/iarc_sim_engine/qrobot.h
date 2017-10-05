#ifndef __QROBOT_H__
#define __QROBOT_H__
#include <memory>

#include "robot.h"
#include "robot_item.h"

class QRobot{
    private:
        std::shared_ptr<Robot> robot;
        std::shared_ptr<RobotItem> item;
    public:
        QRobot(Robot r, RobotItem i);
        void render();
};

#endif
