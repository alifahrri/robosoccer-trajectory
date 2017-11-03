#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "trajectory1d.h"

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
    void printText(QString txt, QColor color = Qt::black, QColor background = Qt::white);

private:
    Ui::MainWindow *ui;
    QCPGraph *pos_graph;
    QCPGraph *vel_graph;
    Dialog *dialog;
    Trajectory1D trajectory;
};

#endif // MAINWINDOW_H
