#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <memory>
#include <functional>
#include <algorithm>

// Qt4
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QTime>

#include "qrobot.h"

namespace Ui{
    class MainWindow;
}


class MainWindow : public QMainWindow{
    using callback_t = std::function<void(float)>;
    // QT
    Q_OBJECT

    private:
        QGraphicsScene* scene;
        Ui::MainWindow *ui;
        std::vector<callback_t> cbs;
        QTime now;
        int timer_id;
        float accel;
    public:
        explicit MainWindow(QWidget* parent, const QPixmap& map);
        ~MainWindow();

        void timerEvent(QTimerEvent*);
        void resizeEvent(QResizeEvent*);
        void add_cb(callback_t cb);

        void spawn(RobotItem* item);
        void kill(RobotItem* item);
    signals:
        void sig_spawn(RobotItem* item);
        void sig_kill(RobotItem* item);
        void sig_reset();
    public slots:
        void handle_spawn(RobotItem* item);
        void handle_kill(RobotItem* item);
        void handle_reset();
        void set_sim_accel(double accel);
    public:
        double get_sim_accel();

};

#endif
