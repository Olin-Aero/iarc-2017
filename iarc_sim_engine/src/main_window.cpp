#include "main_window.h"

MainWindow::MainWindow(QWidget* parent):
    QMainWindow(parent),
    scene(new QGraphicsScene(0,0,512,512,this)),
    ui(new Ui::MainWindow){
        ui->setupUi(this);
        ui->graphicsView->setScene(&scene);
    }

void MainWindow::spawn(std::shared_ptr<QRobot>& robot){
    robots.push_back(robot);
    scene->addItem(robot->item);
}

void MainWindow::kill(const std::string name){
    std::string s;

    auto it = std::find(robots.begin(), robots.end(), [&](const std::shared_ptr<QRobot>& robot){
            robot->robot->get_name(s);
            return s == name;
            });
    if(it != robots.end()){
        scene->removeItem((*it)->item);
        robots.erase(it);
    }
}

void MainWindow::timerEvent(QTimerEvent*){
    _now = QTime::currentTime();
    float dt = now.msecsTo(_now)/1000.0;
    for(auto& cb : cbs){
        cb(dt);
    }
    now = _now;
}
