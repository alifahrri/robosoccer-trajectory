#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "trajectory1d.h"
#include "trajectory2d.h"

namespace Ui {
class MainWindow;
}
class QCPGraph;
class Dialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void computeControl(Trajectory1D::State init, double final, double vmax = 1.0, double amax = 1.0);
    void computeControl2D(Trajectory2D::State2D init, double final_x, double final_y, double vmax = 1.0, double amax = 1.0);
    void printText(QString txt, QColor color = Qt::black, QColor background = Qt::white);

private:
    Ui::MainWindow *ui;
    QCPGraph *pos_graph;
    QCPGraph *vel_graph;
    QCPGraph *pos_graph_y;
    QCPGraph *vel_graph_y;
    Dialog *dialog;
    Trajectory1D::OptimalController trajectory;
    Trajectory2D::OptimalController trajectory2d;
};

#endif // MAINWINDOW_H
