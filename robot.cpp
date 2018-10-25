#include "robot.h"
#include <QPainter>

#define ROBOT_RADIUS 20
#define SCAN_RESOLUTION 6
#define TO_RADIAN M_PI/180.0

Robot::Robot() :
    pos(QPointF(0.0,0.0)),
    angle(0.0),
    robot_color(Qt::red)
{
    auto user = std::getenv("USER");
    auto mat = QTransform();
    mat.rotate(-90);
    pixmap = QPixmap(QString("/home/")+user+"/dev/fukuro.png")
            .scaledToWidth(50,Qt::SmoothTransformation)
            .transformed(mat,Qt::SmoothTransformation);
    pixmap_pt = QPoint(pixmap.rect().width()/2,pixmap.rect().height()/2);
    line_angle.setP1(pos);
    line_angle.setP2(QPoint(ROBOT_RADIUS,ROBOT_RADIUS));
    line_angle.setAngle(angle);
    for(int i=0; i<360; i+=SCAN_RESOLUTION)
    {
        QLineF line(pos,QPointF(0.0,300));
        line.setAngle((double)i+angle);
        scanlines.push_back(line);
    }
}

void Robot::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    auto m =painter->matrix();
    painter->drawLine(line_angle);
    painter->translate(pos);
    painter->rotate(-angle);
    painter->drawPixmap(-pixmap_pt,pixmap);
    painter->setMatrix(m);
    painter->setPen(QPen(Qt::gray));
//    painter->drawLines(scanlines);
    painter->setPen(QPen(robot_color,2.2));
    QVector<QPointF> pts;
    for(size_t i=0; i<tx.size(); i+=50)
        pts.push_back(QPointF(tx.at(i)*100.0,ty.at(i)*100.0));
    painter->drawPoints(pts);
    painter->setBrush(Qt::transparent);
//    painter->drawEllipse(pos,ROBOT_RADIUS,ROBOT_RADIUS);
}

QRectF Robot::boundingRect() const
{
    return QRectF(-450,-300,900,600);
}

void Robot::setPos(double x, double y, double w)
{
    double dx = x-pos.x();
    double dy = y-pos.y();
    pos.setX(x);
    pos.setY(y);
    line_angle.translate(dx,dy);
    line_angle.setAngle(w);
    angle = w;
    double dw = w-angle;
    for(int i=0; i<scanlines.size(); i++)
    {
        scanlines[i].translate(dx,dy);
        scanlines[i].setAngle(scanlines[i].angle()+dw);
    }
    update();
}

void Robot::setVel(double vx, double vy, double w)
{
    double c = cos(angle*TO_RADIAN);
    double s = sin(angle*TO_RADIAN);
    double dx = c*vx+s*vy;
    double dy = -s*vx+c*vy;
    pos.setX(pos.x()+dx);
    pos.setY(pos.y()+dy);
    angle += w;
    line_angle.translate(dx,dy);
    line_angle.setAngle(angle);
    for(int i=0; i<scanlines.size(); i++)
    {
        scanlines[i].translate(dx,dy);
        scanlines[i].setAngle(scanlines[i].angle()+w);
    }
    update();
}

void Robot::setTrajectory(Robot::Vector x, Robot::Vector y, Robot::Vector w, Robot::Vector dx_, Robot::Vector dy_, Robot::Vector dw_)
{
    tx = x;
    ty = y;
    tw = w;
    dx = dx_;
    dy = dy_;
    dw = dw_;
    setPos(tx.first()*100.0,ty.first()*100.0,tw.first());
    update();
}
