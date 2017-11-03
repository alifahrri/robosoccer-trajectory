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
        auto vmax = ui->vmSBox->value();
        auto amax = ui->amSBox->value();
        emit stateChanged(init_pos,init_vel,final_pos,vmax,amax);
    });
}

Dialog::~Dialog()
{
    delete ui;
}
