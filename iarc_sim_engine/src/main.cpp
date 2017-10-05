#include "main_window.h"
#include "qrobot.h"
#include "ros_robot.h"
#include <QApplication>
#include "iarc_sim_engine/SpawnRobot.h"

class ROSInterface{
    private:
        ros::NodeHandle& nh;
        MainWindow& w;
        std::vector<std::shared_ptr<ROSRobot>> robots;

        ros::ServiceServer spawn_srv;
        ros::ServiceServer kill_srv;
        ros::ServiceServer reset_srv;

    public:
        ROSInterface(
                ros::NodeHandle& nh,
                MainWindow& w
                ):nh(nh), w(w){

            spawn_srv = nh.advertiseService("spawn", &ROSInterface::spawn_cb, this);
            kill_srv = nh.advertiseService("kill", &ROSInterface::kill_cb, this);
            reset_srv = nh.advertiseService("reset", &ROSInterface::reset_cb, this);

            w.add_cb(std::bind(&ROSInterface::update, this, _1));
            w.add_cb(std::bind(&ROSInterface::publish, this, _1));
        }

        void spawn_cb(iarc_sim_engine::SpawnRobot::Request& req,
            iarc_sim_engine::SpawnRobot::Response& res){

            float r = req.radius;
            QPointF dims(r,r); // TODO : something more reasonable

            // create ...
            auto robot = std::make_shared<ROSRobot>(nh, req.name);
            robot->set_pos(req.x,req.y,req.t);
            auto item = new RobotItem(dims, req.img);
            item->set_pos(req.x,req.y,req.t);
            auto qrobot = std::make_shared<QRobot>(robot, item);

            // register ...
            robots.push_back(robot);
            w.spawn(qrobot);
        }

        void kill_cb(
                iarc_sim_engine::KillRobot::Request& req,
                iarc_sim_engine::KillRobot::Response& res
                ){
            // delete qrobot
            w.kill(req.name);

            // delete rosrobot
            std::string s;
            auto it = std::find(robots.begin(), robots.end(), [&](const std::shared_ptr<ROSRobot>& robot){
                    robot->get_name(s);
                    return s == req.name;
                    });
            if(it != robots.end()){
                robots.erase(it);
            }
        }

        void reset_cb(
                iarc_sim_engine::ResetRobot::Request& req,
                iarc_sim_engine::ResetRobot::Response& res){
            // not handling for now
        }

        void update(float dt){
            // dt *= ACCEL;
            for(auto& r : robots){
                r->update(dt);
            }
            for(auto& r : robots){
                r->publish();
            }
        }

};

int main(int argc, char* argv[]){
    // ros initializtion
    ros::init(argc, argv, "iarc_sim_engine");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);

    // qt initialization
    QApplication a(argc, argv);
    MainWindow w;

    // ros+qt
    ROSInterface r(nh, w);

    // start running
    spinner.start();
    w.show();
    a.exec();

    // ros shutdown
    ros::waitForShutdown();
}
