#include "main_window.h"
#include "ui_main_window.h"

MainWindow::MainWindow(QWidget* parent):
    QMainWindow(parent),
    scene(new QGraphicsScene(this)),
    ui(new Ui::MainWindow){
        ui->setupUi(this);
        scene->setSceneRect(0, 0, ui->graphicsView->width(), ui->graphicsView->height());
        ui->graphicsView->setScene(scene);
        //ui->graphicsView->scale(1, -1); //bottom left is origin
        //
        timer_id = startTimer(10);

        connect(this, SIGNAL(sig_spawn(RobotItem*)), this, SLOT(handle_spawn(RobotItem*)));
        connect(this, SIGNAL(sig_kill(RobotItem*)), this, SLOT(handle_kill(RobotItem*)));
        accel=1.0;
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
