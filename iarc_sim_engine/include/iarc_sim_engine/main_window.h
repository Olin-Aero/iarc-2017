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
#include <QGraphicsTextItem>

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
        int timer_id;
        float accel;

        QTime ref;
        QTime now;
        double sim_ref;
        QGraphicsTextItem* alt_txt_item;
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
        double get_time();

    // TODO : ugly hack to show drone height
    void show_height(float z);

};

#endif
