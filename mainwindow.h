#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "trajectory1d.h"

namespace Ui {
class MainWindow;
}
class QCPGraph;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void computeControl(Trajectory1D::State init, double final);
    void printText(QString txt, QColor color = Qt::black, QColor background = Qt::white);

private:
    Ui::MainWindow *ui;
    QCPGraph *pos_graph;
    QCPGraph *vel_graph;
    Trajectory1D trajectory;
};

#endif // MAINWINDOW_H
