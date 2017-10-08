#include "main_window.h"
#include "ui_main_window.h"
#include <iostream>

MainWindow::MainWindow(QWidget* parent, const std::string map_s):
    QMainWindow(parent),
    scene(new QGraphicsScene(this)),
    ui(new Ui::MainWindow),
    accel(1.0){
        ui->setupUi(this);

        //fix size
        this->setFixedSize(this->geometry().width(),this->geometry().height());

        auto w = ui->graphicsView->width();
        auto h = ui->graphicsView->height();

        scene->setSceneRect(0, 0, w, h);

        // Add Background
        auto bkpx = QPixmap(QString::fromStdString(map_s)).scaled(w,h);
        auto bk = new QGraphicsPixmapItem(bkpx);

        scene->addItem(bk);

        ui->graphicsView->setScene(scene);

        // update timer
        timer_id = startTimer(10);

        // signals
        connect(this, SIGNAL(sig_spawn(RobotItem*)), this, SLOT(handle_spawn(RobotItem*)));
        connect(this, SIGNAL(sig_kill(RobotItem*)), this, SLOT(handle_kill(RobotItem*)));
    }
MainWindow::~MainWindow(){
    if(scene != nullptr){
        delete scene;
        scene=nullptr;
    }
    if(ui != nullptr){
        delete ui;
        ui=nullptr;
    }
}

void MainWindow::spawn(RobotItem* item){
    emit sig_spawn(item);
}
void MainWindow::kill(RobotItem* item){
    emit sig_kill(item);
}
void MainWindow::handle_spawn(RobotItem* item){
    item->load_img();
    scene->addItem(item); //item ownership goes to scene
}

void MainWindow::handle_kill(RobotItem* item){
    scene->removeItem(item); //item is also deleted
}
void MainWindow::handle_reset(){
    //TODO : implement
}
void MainWindow::timerEvent(QTimerEvent*){
    auto _now = QTime::currentTime();
    float dt = now.msecsTo(_now)/1000.0;
    for(auto& cb : cbs){
        cb(dt);
    }
    now = _now;
    scene->update();
}

void MainWindow::add_cb(callback_t cb){
    cbs.push_back(cb);
}

void MainWindow::set_sim_accel(double _accel){
    accel=_accel;
}
double MainWindow::get_sim_accel(){
    return accel;
}
