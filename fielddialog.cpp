#include "fielddialog.h"
#include "ui_fielddialog.h"
#include <QResizeEvent>

FieldDialog::FieldDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FieldDialog),
    field(new Field),
    robot(new Robot)
{
    ui->setupUi(this);
    auto s = this->size();
    auto w = s.width();
    auto h = s.height();
    auto scene = new QGraphicsScene(-w/2,-h/2,w,h,this);
    ui->graphicsView->setScene(scene);
    scene->addItem(field);
    scene->addItem(robot);
    scene->setBackgroundBrush(Qt::darkGreen);
    ui->graphicsView->setRenderHints(QPainter::Antialiasing |
                                     QPainter::HighQualityAntialiasing |
                                     QPainter::SmoothPixmapTransform);
}

void FieldDialog::setCallback(Field::Callback cb)
{
    field->callback = cb;
}

void FieldDialog::drawTrajectory(vec_double tx, vec_double ty, vec_double tw, vec_double dx, vec_double dy, vec_double dw)
{
    robot->setTrajectory(tx,ty,tw,dx,dy,dw);
}

FieldDialog::~FieldDialog()
{
    delete ui;
}

void FieldDialog::resizeEvent(QResizeEvent *event)
{
    auto s = event->size();
    auto w = s.width();
    auto h = s.height();
    auto scene = ui->graphicsView->scene();
    scene->setSceneRect(-w/2,-h/2,w,h);
}
