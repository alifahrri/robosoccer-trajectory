#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dialog.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    dialog(new Dialog(this))
{
    ui->setupUi(this);
    pos_graph = ui->widget->addGraph();
    vel_graph = ui->widget->addGraph();
    pos_graph_y = ui->widget->addGraph();
    vel_graph_y = ui->widget->addGraph();
    vel_graph->setPen(QPen(Qt::red));
    pos_graph_y->setPen(QPen(Qt::blue,1.0,Qt::DashLine));
    vel_graph_y->setPen(QPen(Qt::red,1.0,Qt::DashLine));
//    computeControl({0.0,0.5},3.0);
    computeControl2D({{0.0,0.0},{0.0,0.0}},3.0,3.0);
    setWindowTitle(QString("Optimal Control Test"));
    connect(dialog,&Dialog::stateChanged,[=](double x0, double vx0, double xf, double y0, double vy0, double yf, double vmax, double amax)
    {
//        computeControl({x0,v0},xf,vmax,amax);
        computeControl2D({{x0,vx0},{y0,vy0}},xf,yf,vmax,amax);
    });
    connect(ui->actionDialog,&QAction::toggled,[=](bool checked)
    {
        if(checked && !dialog->isVisible())
            dialog->show();
        else if(!checked && dialog->isVisible())
            dialog->hide();
    });
    connect(dialog,&QDialog::finished,[=]
    {
        ui->actionDialog->setChecked(false);
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

inline
void MainWindow::computeControl(Trajectory1D::State init, double final, double vmax, double amax)
{
    printText("computing",Qt::green,Qt::darkGreen);
    double final_time = 0.0;
    trajectory.setLimit(vmax,amax);
    Trajectory1D::OptimalController::ControlSequence ctrl;
    try {
        ctrl = trajectory.optimalControl(init,final,final_time);
        final_time = Trajectory1D::OptimalController::setMaxEffort(ctrl,2.0);
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << '\n';
        exit(-1);
    }

    printText("done : ");
    for(auto c : ctrl)
    {
        switch(c.control_case)
        {
        case Trajectory1D::ACCELERATION1 :
            printText("init velocity negative..");
            break;
        case Trajectory1D::ACCELERATION2_2 :
        case Trajectory1D::ACCELERATION2_1 :
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
    double time_step = 0.001;
    int time_count = (int)(final_time/time_step);
    QVector<double> keys;
    QVector<double> pos_values;
    QVector<double> vel_values;
//    for(int i=0; i<time_count; i++)
//    {
//        keys.push_back(i*time_step);
//        auto s = Trajectory1D::getState(ctrl,i*time_step);
//        pos_values.push_back(s.w);
//        vel_values.push_back(s.dw);
//    }
//    pos_graph->setData(keys,pos_values);
//    vel_graph->setData(keys,vel_values);
    auto trajectory = Trajectory1D::OptimalController::getTrajectory(ctrl,0.0,final_time,time_step);
    for(auto s : trajectory.first)
    {
        pos_values.push_back(s.w);
        vel_values.push_back(s.dw);
    }
    pos_graph->setData(QVector<double>::fromStdVector(trajectory.second),pos_values);
    vel_graph->setData(QVector<double>::fromStdVector(trajectory.second),vel_values);
    ui->widget->xAxis->rescale(true);
    ui->widget->yAxis->rescale(true);
    ui->widget->replot();
    printText(QString("time : %1").arg(final_time));
}

void MainWindow::computeControl2D(Trajectory2D::State2D init, double final_x, double final_y, double vmax, double amax)
{
    trajectory2d.setLimit(vmax,amax);
    auto ctrl = trajectory2d.optimalControl(init,final_x,final_y);
    const auto& x_ctrl = ctrl.x;
    const auto& y_ctrl = ctrl.y;
    auto time_step(0.001);
    auto trajectory_x = Trajectory1D::OptimalController::getTrajectory(x_ctrl,0,x_ctrl.final_time,time_step);
    auto trajectory_y = Trajectory1D::OptimalController::getTrajectory(y_ctrl,0,y_ctrl.final_time,time_step);
    QVector<double> keys;
    QVector<double> pos_x;
    QVector<double> vel_x;
    QVector<double> pos_y;
    QVector<double> vel_y;
    for(auto s : trajectory_x.first)
    {
        pos_x.push_back(s.w);
        vel_x.push_back(s.dw);
    }
    for(auto s : trajectory_y.first)
    {
        pos_y.push_back(s.w);
        vel_y.push_back(s.dw);
    }
    pos_graph->setData(QVector<double>::fromStdVector(trajectory_x.second),pos_x);
    vel_graph->setData(QVector<double>::fromStdVector(trajectory_x.second),vel_x);
    pos_graph_y->setData(QVector<double>::fromStdVector(trajectory_y.second),pos_y);
    vel_graph_y->setData(QVector<double>::fromStdVector(trajectory_y.second),vel_y);
    ui->widget->xAxis->rescale(true);
    ui->widget->yAxis->rescale(true);
    ui->widget->replot();
}

inline
void MainWindow::printText(QString txt, QColor color, QColor background)
{
    ui->textEdit->setTextColor(color);
    ui->textEdit->setTextBackgroundColor(background);
    ui->textEdit->append(txt);
}
