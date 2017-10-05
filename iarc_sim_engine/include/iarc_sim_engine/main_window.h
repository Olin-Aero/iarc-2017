#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <memory>
// Qt4
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QTime>

#include "qrobot.h"

namespace Ui{
    class MainWindow;
}

using callback_t = std::function<void(float)>;

class MainWindow : public QMainWindow{
    Q_OBJECT
    private:
        QGraphicsScene scene;
        std::vector<std::shared_ptr<QRobot>> robots;
        Ui::MainWindow *ui;
        QTime now;
        std::vector<callback_t> cbs;
    public:
        explicit MainWindow(QWidget* parent=0);
        ~MainWindow();

        void spawn(std::shared_ptr<QRobot>& robot);
        void kill(const std::string name);
        void reset();

        void timerEvent(QTimerEvent*);
        void add_cb(callback_t cb);


            // add timer event
            // void add_cb();

};

#endif
