#include <algorithm>
#include <functional>
#include <iostream>

#include "main_window.h"
#include "qrobot.h"
#include "ros_robot.h"
#include <QApplication>
#include <csignal>

#include "iarc_sim_engine/SpawnRobot.h"
#include "iarc_sim_engine/KillRobot.h"

class ROSInterface{
    private:
        ros::NodeHandle& nh;
        ros::NodeHandle& nh_priv;

        MainWindow& w;
        std::vector<std::shared_ptr<QRobot>> robots;

        ros::ServiceServer spawn_srv;
        ros::ServiceServer kill_srv;
        ros::ServiceServer reset_srv;
    public:
        ROSInterface(
                ros::NodeHandle& nh,
                ros::NodeHandle& nh_priv,
                MainWindow& w
                ):nh(nh),nh_priv(nh_priv),w(w){

            spawn_srv = nh.advertiseService("spawn", &ROSInterface::spawn_cb, this);
            kill_srv = nh.advertiseService("kill", &ROSInterface::kill_cb, this);
            //reset_srv = nh.advertiseService("reset", &ROSInterface::reset_cb, this);
            w.add_cb([&](float dt){return this->update(dt);});
        }

        bool spawn_cb(
                iarc_sim_engine::SpawnRobot::Request& req,
                iarc_sim_engine::SpawnRobot::Response& res){

            std::string s;
            for(auto& r : robots){
                r->robot->get_name(s);
                if (req.name == s){
                    ROS_INFO("Robot Name %s Already Exists", s.c_str());
                    res.success = false;
                    return false;
                }
            }

            float r = req.radius;
            QPointF dims(r,r); // TODO : something more reasonable

            // create ...
            auto robot = std::make_shared<ROSRobot>(nh, req.name);
            robot->set_pos(req.x,req.y,req.t);
            auto item = new RobotItem(dims, QString::fromStdString(req.img));
            w.spawn(item);

            QPointF pos(req.x, req.y);
            item->set_pos(pos, req.t);
            auto qrobot = std::make_shared<QRobot>(robot, item);

            // register ...
            robots.push_back(qrobot);
            res.success = true;
            return true;
        }

        bool kill_cb(
                iarc_sim_engine::KillRobot::Request& req,
                iarc_sim_engine::KillRobot::Response& res
                ){
            // delete rosrobot
            std::string s;
            auto it = std::find_if(robots.begin(), robots.end(), [&](const std::shared_ptr<QRobot>& r){
                    r->robot->get_name(s);
                    return (s == req.name);
                    });
            if(it != robots.end()){
                w.kill((*it)->item);
                robots.erase(it);
                res.success = true;
                return true;
            }

            //failed to kill
            ROS_INFO("Robot Name %s Does not Exist", req.name.c_str());
            return false;
        }

        //void reset_cb(
        //        iarc_sim_engine::ResetRobot::Request& req,
        //        iarc_sim_engine::ResetRobot::Response& res){
        //    // not handling for now
        //}
        
        void update(float dt){
            // dt *= ACCEL;
            for(auto& r : robots){
                r->update(w.get_sim_accel() * dt);
            }
            for(auto& r : robots){
                dynamic_cast<ROSRobot&>(*(r->robot)).publish();
            }
        }

};

QApplication* a_ptr=nullptr;

void handle_signal(int sig){
    if(a_ptr){
        a_ptr->quit();
        a_ptr = nullptr;
    }
}

int main(int argc, char* argv[]){
    // ros initializtion
    ros::init(argc, argv, "iarc_sim_engine");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~"); //private node handle
    ros::AsyncSpinner spinner(4);

    // load parameters
    std::string map_file;
    double map_dim;

    nh_priv.param<std::string>("map", map_file, "");
    nh_priv.param<double>("map_dim", map_dim, 20.0);

    // qt initialization
    QApplication a(argc, argv);
    a_ptr = &a;

    QPixmap map(QString::fromStdString(map_file));
    MAP_M = map_dim; // remember map dimensions
    MAP_PX = map.width(); // this is from image

    MainWindow w(nullptr, map);
    std::signal(SIGINT, &handle_signal);

    // ros+qt
    ROSInterface r(nh, nh_priv, w);

    // start running
    spinner.start();
    w.show();
    a.exec();

    // ros shutdown
    ros::shutdown();
}
