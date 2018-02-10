#include "main_window.h"
#include "ui_main_window.h"
#include "utils.h"
#include <iostream>

MainWindow::MainWindow(QWidget* parent, const QPixmap& map):
    QMainWindow(parent),
    scene(new QGraphicsScene(this)),
    ui(new Ui::MainWindow),
    accel(1.0){

        ui->setupUi(this);
        auto view = ui->graphicsView;

        // fix size for now
        this->setFixedSize(this->geometry().width(),this->geometry().height());

        auto w = map.width();
        auto h = map.height();

        scene->setSceneRect(0, 0, w, h);
        view->setSceneRect(0, 0, w, h);

        // Add Background
        auto bk = new QGraphicsPixmapItem(map);
        scene->addItem(bk);

        // Add Altitude
        alt_txt_item = new QGraphicsTextItem("0.0");
        alt_txt_item->setDefaultTextColor(QColor::fromRgbF(1.0,1.0,1.0));
        scene->addItem(alt_txt_item);
        
        view->setScene(scene);
        view->scale(float(view->width())/map.width(), float(view->height())/map.height());

        // simulation time
        ref = QTime::currentTime();
        now = QTime::currentTime();
        sim_ref = 0.0;

        // update timer
        timer_id = startTimer(5);

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
    float dt = accel * now.msecsTo(_now)/1000.0;
    for(auto& cb : cbs){
        cb(dt);
    }
    now = _now;
    scene->update();
}
void MainWindow::resizeEvent(QResizeEvent*){
    // TODO: scaleable
}
void MainWindow::add_cb(callback_t cb){
    cbs.push_back(cb);
}

void MainWindow::set_sim_accel(double _accel){
    // update reference
    sim_ref += accel * ref.msecsTo(QTime::currentTime())/1000.0;
    ref = QTime::currentTime();

    accel =_accel;
}
double MainWindow::get_sim_accel(){
    return accel;
}
double MainWindow::get_time(){
    auto _now = QTime::currentTime();
    return sim_ref + accel * ref.msecsTo(_now)/1000.0;
}

// TODO : ugly hack to display altitude
void MainWindow::show_height(float z){
    alt_txt_item->setPlainText(QString::number(z));
}
