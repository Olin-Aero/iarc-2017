#include "main_window.h"

MainWindow::MainWindow(QWidget* parent):
    QMainWindow(parent),
    scene(new QGraphicsScene(0,0,512,512,this)),
    ui(new Ui::MainWindow){
        ui->setupUi(this);
        ui->graphicsView->setScene(&scene);
    }

void MainWindow::timerEvent(QTimerEvent*){
    _now = QTime::currentTime();
    float dt = now.msecsTo(_now)/1000.0;
    for(auto& cb : cbs){
        cb(dt);
    }
    now = _now;
}
