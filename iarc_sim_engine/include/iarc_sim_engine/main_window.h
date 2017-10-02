#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

// Qt4
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>

// ROS
#include <ros/ros.h>
#include "ros_robot.h"

namespace Ui{
    class MainWindow;
}

class MainWindow : public QMainWindow{
    Q_OBJECT
private:
    QGraphicsScene scene;
    std::vector<ROSRobot> robots;
    Ui::MainWindow *ui;
public:
    explicit MainWindow(QWidget* parent=0);
    ~MainWindow();
    void reset();
    void timerEvent(QTimerEvent*);

    void subscribe();
    void publish();
};

#endif
