#ifndef FIELDDIALOG_H
#define FIELDDIALOG_H

#include <QDialog>
#include <functional>
#include "field.h"
#include "robot.h"

namespace Ui {
class FieldDialog;
}

class FieldDialog : public QDialog
{
    Q_OBJECT

    typedef QVector<double> vec_double;
public:
    explicit FieldDialog(QWidget *parent = 0);
    void setCallback(Field::Callback cb);
    void drawTrajectory(vec_double tx, vec_double ty, vec_double tw, vec_double dx, vec_double dy, vec_double dw);
    ~FieldDialog();
protected:
    void resizeEvent(QResizeEvent *event);
private:
    Ui::FieldDialog *ui;
    Field *field;
    Robot *robot;
};

#endif // FIELDDIALOG_H
