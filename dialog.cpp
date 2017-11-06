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
        auto vmax = ui->vmSBox->value();
        auto amax = ui->amSBox->value();
        emit stateChanged(init_pos,init_vel,final_pos,init_pos_y,init_vel_y,final_pos_y,vmax,amax);
    });
}

Dialog::~Dialog()
{
    delete ui;
}
