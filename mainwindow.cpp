#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    computeControl({0.0,0.5},3.0);
    graph = ui->widget->addGraph();
}

MainWindow::~MainWindow()
{
    delete ui;
}

inline
void MainWindow::computeControl(Trajectory1D::State init, double final)
{
    printText("computing",Qt::yellow);
    auto ctrl = trajectory.optimalControl(init,final);
    printText("done :");
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
    }
}

inline
void MainWindow::printText(QString txt, QColor color, QColor background)
{
    ui->textEdit->setTextColor(color);
    ui->textEdit->setTextBackgroundColor(background);
    ui->textEdit->append(txt);
}
