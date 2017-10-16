#ifndef __ROBOTITEM_H__
#define __ROBOTITEM_H__

#include <QObject>
#include <QPointF>
#include <QRectF>
#include <QWidget>
#include <QPainter>
#include <QGraphicsItem>
#include <QPixmap>

#include "utils.h"

class RobotItem : public QGraphicsItem{
private:
    QPointF pos;
    float theta;
    QPointF dims;
    QString img_s;
    QPixmap* img;
    //frequently requested derived features
public:
    RobotItem(const QPointF dims, const QString img_s);
    ~RobotItem();
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void load_img();
    void set_pos(QPointF pos, float theta);
};

#endif
