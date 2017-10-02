MainWindow::MainWindow(QWidget* parent):
    QMainWindow(parent),
    scene(new QGraphicsScene(0,0,512,512,this)),
    ui(new Ui::MainWindow){
        ui->setupUi(this);
        ui->graphicsView->setScene(&scene);
    }

void MainWindow::timerEvent(QTimerEvent*){
    publish();
}

void MainWindow::publish(){
    float x,y,t;
    for(auto& r : robots){
        r.publish();
    }
}
