#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dialog.h"
#include "fielddialog.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    dialog(new Dialog(this)),
    field_dialog(new FieldDialog(this))
{
    ui->setupUi(this);
    pos_graph = ui->widget->addGraph();
    vel_graph = ui->widget->addGraph();
    pos_graph_y = ui->widget->addGraph();
    vel_graph_y = ui->widget->addGraph();
    pos_graph_w = ui->widget->addGraph(ui->widget->xAxis,ui->widget->yAxis2);
    vel_graph_w = ui->widget->addGraph(ui->widget->xAxis,ui->widget->yAxis2);
    ui->widget->yAxis2->setVisible(true);
    pos_graph->setPen(QPen(Qt::blue,1.0));
    vel_graph->setPen(QPen(Qt::blue,1.0,Qt::DashLine));
    pos_graph_y->setPen(QPen(Qt::red,1.0));
    vel_graph_y->setPen(QPen(Qt::red,1.0,Qt::DashLine));
    pos_graph_w->setPen(QPen(Qt::green,1.0));
    vel_graph_w->setPen(QPen(Qt::green,1.0,Qt::DashLine));
    setWindowTitle(QString("Optimal Control Test"));
    connect(dialog,&Dialog::stateChanged,[=](double x0, double vx0, double xf, double y0, double vy0, double yf, double w0, double vw0, double wf, double vmax, double amax, double vmax_w, double amax_w)
    {
        controlRobot({x0,y0,w0,vx0,vy0,vw0},{xf,yf,wf,0.0,0.0,0.0},vmax,amax,vmax_w,amax_w);
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
    connect(ui->actionField,&QAction::toggled,[=](bool checked)
    {
        if(checked && !field_dialog->isVisible())
            field_dialog->show();
        else if(!checked && field_dialog->isVisible())
            field_dialog->hide();
    });
    connect(field_dialog,&QDialog::finished,[=]
    {
        ui->actionField->setChecked(false);
    });
    field_dialog->setCallback([=](Field::State init, Field::State final)
    {
        dialog->setState(init.x,init.y,init.w,init.dx,init.dy,init.dw,final.x,final.y,final.w);
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
    Trajectory1D::Controller::Control ctrl;
    try {
        ctrl = trajectory.optimalControl(init,final,final_time);
        final_time = Trajectory1D::Controller::setMaxEffort(ctrl,2.0);
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
    auto trajectory = Trajectory1D::Controller::getTrajectory(ctrl,0.0,final_time,time_step);
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
    auto trajectory_x = Trajectory1D::Controller::getTrajectory(x_ctrl,0,x_ctrl.final_time,time_step);
    auto trajectory_y = Trajectory1D::Controller::getTrajectory(y_ctrl,0,y_ctrl.final_time,time_step);
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

void MainWindow::controlRobot(RobotTrajectory::State init, RobotTrajectory::State final, double vlin, double alin, double vang, double aang)
{
    auto dt(0.001);
    auto tf_lin(0.0);
    auto tf_ang(0.0);
    generator.setLimit(vlin,alin,vang,aang);
    generator.generate(init,final,&tf_lin,&tf_ang);
    qDebug() << "tf :" << tf_lin << tf_ang << std::max(tf_lin,tf_ang);
    auto trajectory = generator.getTrajectory(0.0,std::max(tf_lin,tf_ang),dt);
    QVector<double> pos_x;
    QVector<double> vel_x;
    QVector<double> pos_y;
    QVector<double> vel_y;
    QVector<double> pos_w;
    QVector<double> vel_w;
    auto time = QVector<double>::fromStdVector(trajectory.second);
    for(auto t : trajectory.first)
    {
        pos_x.push_back(t.x);
        pos_y.push_back(t.y);
        pos_w.push_back(t.w);
        vel_x.push_back(t.dx);
        vel_y.push_back(t.dy);
        vel_w.push_back(t.dw);
    }
    pos_graph->setData(time,pos_x);
    vel_graph->setData(time,vel_x);
    pos_graph_y->setData(time,pos_y);
    vel_graph_y->setData(time,vel_y);
    pos_graph_w->setData(time,pos_w);
    vel_graph_w->setData(time,vel_w);
    ui->widget->yAxis->rescale(true);
    ui->widget->yAxis2->rescale(true);
    ui->widget->xAxis->rescale(true);
    ui->widget->replot();
    field_dialog->drawTrajectory(pos_x,pos_y,pos_w,vel_x,vel_y,vel_w);
}

inline
void MainWindow::printText(QString txt, QColor color, QColor background)
{
    ui->textEdit->setTextColor(color);
    ui->textEdit->setTextBackgroundColor(background);
    ui->textEdit->append(txt);
}
