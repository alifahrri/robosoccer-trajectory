#include "dialog.h"
#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    connect(this->ui->pushButton,&QPushButton::clicked,[=]
    {
        auto init_pos = ui->initPosSBox->value();
        auto init_vel = ui->initVelSBox->value();
        auto final_pos = ui->finalPosSBox->value();
        auto init_pos_y = ui->initPosSBoxY->value();
        auto init_vel_y = ui->initVelSBoxY->value();
        auto final_pos_y = ui->finalPosSBoxY->value();
        auto init_pos_w = ui->initPosSBoxW->value();
        auto init_vel_w = ui->initVelSBoxW->value();
        auto final_pos_w = ui->finalPosSBoxW->value();
        auto vmax = ui->vmSBox->value();
        auto amax = ui->amSBox->value();
        auto vmax_w = ui->vmSBoxW->value();
        auto amax_w = ui->amSBoxW->value();
        emit stateChanged(init_pos,init_vel,final_pos,init_pos_y,init_vel_y,final_pos_y,init_pos_w,init_vel_w,final_pos_w,vmax,amax,vmax_w,amax_w);
    });
}

void Dialog::setState(double x, double y, double w, double vx, double vy, double vw, double xf, double yf, double wf)
{
    ui->initPosSBox->setValue(x);
    ui->initPosSBoxY->setValue(y);
    ui->initPosSBoxW->setValue(w);
    ui->initVelSBox->setValue(vx);
    ui->initVelSBoxY->setValue(vy);
    ui->initVelSBoxW->setValue(vw);
    ui->finalPosSBox->setValue(xf);
    ui->finalPosSBoxY->setValue(yf);
    ui->finalPosSBoxW->setValue(wf);
    ui->pushButton->click();
}

Dialog::~Dialog()
{
    delete ui;
}
