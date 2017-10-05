#include "robot_item.h"

RobotItem::RobotItem(QPointF dims, std::string img):img(img){

}

QRectF RobotItem::boundingRect() const{
    return QRectF(pos - dims/2, pos + dims/2);
}

void RobotItem::set_pos(QPointF pos, float t){
    pos = p;
    theta = t;
}

void RobotItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, int* widget){
    painter->translate(pos);
    painter->rotate(-theta);
    painter->drawPixmap(img);
}
