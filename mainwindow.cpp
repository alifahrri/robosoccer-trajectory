#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pos_graph = ui->widget->addGraph();
    vel_graph = ui->widget->addGraph();
    vel_graph->setPen(QPen(Qt::red));
    computeControl({0.0,0.5},3.0);
    setWindowTitle(QString("Optimal Control Test"));
}

MainWindow::~MainWindow()
{
    delete ui;
}

inline
void MainWindow::computeControl(Trajectory1D::State init, double final)
{
    printText("computing",Qt::green,Qt::darkGreen);
    double final_time = 0.0;
    auto ctrl = trajectory.optimalControl(init,final,final_time);
    printText("done : ");
    for(auto c : ctrl)
    {
        switch(c.control_case)
        {
        case Trajectory1D::ACCELERATION1 :
            printText("init velocity negative..");
            break;
        case Trajectory1D::ACCELERATION2 :
            printText("too slow or far away..");
            break;
        case Trajectory1D::CRUISING :
            printText("cruising..");
            break;
        case Trajectory1D::DECELERATION1 :
            printText("braking..");
            break;
        case Trajectory1D::DECELERATION2 :
            printText("too fast, slowing down..");
            break;
        }
        QString str;
        str.append(QString("time : %1\n").arg(c.time));
        str.append(QString("distance : %1\n").arg(c.term.w));
        str.append(QString("velocity : %1\n").arg(c.term.dw));
        str.append(QString("effort : %1\n").arg(c.effort));
        printText(str);
    }
    double time_step = 0.01;
    int time_count = (int)(final_time/time_step);
    QVector<double> keys;
    QVector<double> pos_values;
    QVector<double> vel_values;
    for(int i=0; i<time_count; i++)
    {
        keys.push_back(i*time_step);
        auto s = Trajectory1D::getState(ctrl,i*time_step);
        pos_values.push_back(s.w);
        vel_values.push_back(s.dw);
    }
    pos_graph->setData(keys,pos_values);
    vel_graph->setData(keys,vel_values);
    ui->widget->xAxis->rescale(true);
    ui->widget->replot();
    printText(QString("time : %1").arg(final_time));
}

inline
void MainWindow::printText(QString txt, QColor color, QColor background)
{
    ui->textEdit->setTextColor(color);
    ui->textEdit->setTextBackgroundColor(background);
    ui->textEdit->append(txt);
}
