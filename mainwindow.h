#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "trajectory1d.h"
#include "trajectory2d.h"
#include "trajectoryplanner.h"

namespace Ui {
class MainWindow;
}
class QCPGraph;
class Dialog;
class FieldDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void computeControl(Trajectory1D::State init, double final, double vmax = 1.0, double amax = 1.0);
    void computeControl2D(Trajectory2D::State2D init, double final_x, double final_y, double vmax = 1.0, double amax = 1.0);
    void controlRobot(RobotTrajectory::State init, RobotTrajectory::State final, double vlin = 1.0, double alin = 1.0, double vang = 1.0, double aang = 1.0);
    void printText(QString txt, QColor color = Qt::black, QColor background = Qt::white);

private:
    Ui::MainWindow *ui;
    QCPGraph *pos_graph;
    QCPGraph *vel_graph;
    QCPGraph *pos_graph_y;
    QCPGraph *vel_graph_y;
    QCPGraph *pos_graph_w;
    QCPGraph *vel_graph_w;
    Dialog *dialog;
    FieldDialog *field_dialog;
    Trajectory1D::Controller trajectory;
    Trajectory2D::Controller trajectory2d;
    RobotTrajectory::Generator generator;
};

#endif // MAINWINDOW_H
