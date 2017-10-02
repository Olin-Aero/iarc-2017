#ifndef __ROBOTITEM_H__
#define __ROBOTITEM_H__

#include <QObject>
#include <QPointF>
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
    QPixmap img;
    //frequently requested derived features
    float cr;
public:
    RobotItem(QPointF pos, QPointF dims, std::string img);
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void set_pos(QPointF pos, float theta);
};

#endif


