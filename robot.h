#ifndef ROBOT_H
#define ROBOT_H

#include <QGraphicsItem>
#include <QPixmap>

class Robot : public QGraphicsItem
{
    typedef QVector<double> Vector;
public:
    Robot();
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const;
    void setPos(double x, double y, double w);
    void setVel(double vx, double vy, double w);
    void setColor(QColor color) { robot_color = color; }
    void setTrajectory(Vector x, Vector y, Vector w, Vector dx_, Vector dy_, Vector dw_);
    double getAngle() { return angle; }
    QVector<QLineF> getScanlines() { return scanlines; }
    QPointF getPos() { return pos; }

private:
    QPointF pos;
    double angle;
    QLineF line_angle;
    QVector<QLineF> scanlines;
    QColor robot_color;
    QPixmap pixmap;
    QPoint pixmap_pt;
    QVector<double> tx, ty, tw, dx, dy, dw;
};

#endif // ROBOT_H
