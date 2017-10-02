#include "main_window.h"
#include <QApplication>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "iarc_sim_engine");
    ros::NodeHandle nh;
    QApplication app(argc, argv);

    MainWindow w;
    w.show();
    return a.exec();
}
