#include "robot_item.h"
#include <iostream>

RobotItem::RobotItem(const QPointF dims, const QString img_s)
    :dims(dims), img_s(img_s){

}
RobotItem::~RobotItem(){
    if(img != nullptr){
        delete img;
        img=nullptr;
    }
}
QRectF RobotItem::boundingRect() const{
    int w = m2p(dims.x());
    int h = m2p(dims.y());
    return QRectF(-w/2,-h/2,w,h);
}

void RobotItem::set_pos(QPointF p, float t){
    pos = p;
    theta = t;
}

void RobotItem::load_img(){
    img = new QPixmap(img_s);
}

QPointF m2p(const QPointF m){
    return QPointF(m2p(m.x()), m2p(m.y()));
}

QPointF p2m(const QPointF p){
    return QPointF(p2m(p.x()), p2m(p.y()));
}

void RobotItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget){
    painter->setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform, true);
    
    int ww = widget->size().width();
    int wh = widget->size().height();
    //std::cout << ww << "," << wh  << std::endl;
        
    int w = img->width();
    int h = img->height();

    QTransform xform;
    xform.translate(m2p(pos.x()), -m2p(pos.y()));
    xform.rotate(r2d(-theta));
    xform.scale(m2p(dims.x())/w, m2p(dims.y())/h);

    painter->setWindow(QRect(-ww/2,-wh/2,ww,wh));
    painter->setTransform(xform, true);

    QPointF dp(-w/2.0, -h/2.0);
    painter->drawPixmap(dp, *img);
    //painter->drawRect(QRectF(w*-dims/2.0, h*dims/2.0));
    //painter->drawEllipse(pos, m2p(0.7)/w, m2p(0.7)/w);
    painter->resetTransform();
}
