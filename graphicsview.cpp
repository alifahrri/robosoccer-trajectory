#include "graphicsview.h"
#include <QKeyEvent>

GraphicsView::GraphicsView(QWidget *parent) :
    QGraphicsView(parent)
{

}

void GraphicsView::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_W:
        emit keyPressed('w');
        break;
    case Qt::Key_A:
        emit keyPressed('a');
        break;
    case Qt::Key_S:
        emit keyPressed('s');
        break;
    case Qt::Key_D:
        emit keyPressed('d');
        break;
    case Qt::Key_Q:
        emit keyPressed('q');
        break;
    case Qt::Key_E:
        emit keyPressed('e');
        break;
    case Qt::Key_Plus:
        this->scale(1.1,1.1);
        break;
    case Qt::Key_Minus:
        this->scale(0.9,0.9);
        break;
    default:
        break;
    }
}
