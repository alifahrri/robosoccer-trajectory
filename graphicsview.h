#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include <QGraphicsView>

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    GraphicsView(QWidget *parent=nullptr);
private:
    void keyPressEvent(QKeyEvent *event);
signals:
    void keyPressed(char);
};

#endif // GRAPHICSVIEW_H
