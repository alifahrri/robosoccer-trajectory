#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    void setState(double x, double y, double w, double vx, double vy, double vw, double xf, double yf, double wf);
    ~Dialog();

private:
    Ui::Dialog *ui;

signals:
    void stateChanged(double,double,double,double,double,double,double,double,double,double,double,double,double);
};

#endif // DIALOG_H
