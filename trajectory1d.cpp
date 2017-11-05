#include "trajectory1d.h"
#include <cmath>
#include <iostream>
#include <QDebug>
#include <stdexcept>
#include <sstream>
#define sign(X) (X<0?-1.0:1.0)

Trajectory1D::Trajectory1D()
    : a_max(1.0),
      v_max(1.0)
{

}

void Trajectory1D::setLimit(double vmax, double amax)
{
    v_max = vmax;
    a_max = amax;
}

Trajectory1D::ControlSequence Trajectory1D::optimalControl(Trajectory1D::State init_state, double final_state, double &final_time)
{
    ControlSequence ctrl_seq;
    State initial_state = init_state;
    Control ctrl;
    ctrl.term = init_state;
    ctrl.time = 0.0;
    ctrl_seq.push_back(ctrl);
    ctrl_seq.offset = init_state.w < 0.0 ? fabs(init_state.w) : 0.0;
    ctrl_seq.distance = 0.0;
    double t = 0.0;
    double wf = final_state - initial_state.w; //shifted final
apply_control :
    double w_diff = (final_state) - initial_state.w;
    if((fabs(w_diff) <= 0.00001) && (fabs(initial_state.dw) <= 0.001))
        goto done;
    applyControl(ctrl_seq,initial_state,final_state);

    t += fabs(ctrl_seq.back().time);
    initial_state = ctrl_seq.back().term;
//    qDebug() << "wf :" << wf << "initial state :" << QString("[%1,%2]").arg(initial_state.w).arg(initial_state.dw) << "w_diff :" << w_diff;
    goto apply_control;
done:
    final_time = t;
    ctrl_seq.final_time = t;
    return ctrl_seq;
}

Trajectory1D::State Trajectory1D::getState(const Trajectory1D::ControlSequence &ctrl, double time)
{
    State s;
    double t = 0.0;
    double dt = time;
    for(int i=1; i<ctrl.size(); ++i)
    {
        auto c = ctrl.at(i);
        t += c.time;
        auto s0 = ctrl.at(i-1).term; // prev state
        if(time < t)
        {
            s.dw = s0.dw + c.effort*dt;
            s.w = s0.w + s0.dw*dt + c.effort*dt*dt/2.0;
            break;
        }
        dt -= c.time;
    }
    return s;
}

std::__cxx11::string Trajectory1D::str(Trajectory1D::State &s)
{
    std::stringstream str;
    str << "[" << s.w << "," << s.dw << "]";
    return str.str();
}

inline
void Trajectory1D::applyControl(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    double wf = final - init_state.w;
    double normal_wf = wf*sign(wf);
    double w0 = 0.0;
    double dw0 = init_state.dw*sign(wf); //normalized dw0
    double tmp = dw0*dw0/(2.0*a_max);
    State initial_state = {w0,dw0};
    qDebug() << "apply control :" << "final :" << final
             << "wf :" << wf
             << "normal wf :" << normal_wf
             << "tmp :" << tmp
             << "init_state :" << str(init_state).data()
             << "normalwf == tmp?" << (normal_wf==tmp?"true;" : "false;")
             << "normalwf > tmp?" << (normal_wf>tmp?"true;" : "false;")
             << "normalwf < tmp?" << (normal_wf<tmp?"true;" : "false;");
    double wf_diff = normal_wf-tmp;
    if(dw0<0)
        case1(ctrl_seq,initial_state,normal_wf);
    else if(dw0<v_max /*&& dw0>=0.0*/ && wf_diff>0.00001)
        case21(ctrl_seq,initial_state,normal_wf);
    else if(dw0==v_max && wf_diff>0.00001)
        case22(ctrl_seq,initial_state,normal_wf);
    else if(dw0<=v_max /*&& dw0>0.0*/ && wf_diff<=0.00001)
        case23(ctrl_seq,initial_state,normal_wf);
    else if(dw0>v_max)
        case3(ctrl_seq,initial_state,normal_wf);
    else
    {
        throw std::runtime_error("case not match!");
    }
    ctrl_seq.back().effort *= sign(wf);
    ctrl_seq.back().term.dw *= sign(wf);
    ctrl_seq.back().term.w *= sign(wf);
    ctrl_seq.back().term.w += init_state.w;
    ctrl_seq.distance = ctrl_seq.back().term.w;
    if(ctrl_seq.size()>100)
        throw std::runtime_error("control sequence error!");
}

inline
void Trajectory1D::case1(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Control ctrl;
    ctrl.control_case = ACCELERATION1;
    ctrl.effort = a_max;
    ctrl.time = -init_state.dw/a_max;
    ctrl.term.w = -init_state.dw*init_state.dw/(2.0*a_max);
    ctrl.term.dw = 0;
    ctrl_seq.push_back(ctrl);
    std::ostringstream ctrl_str;
    ctrl_str << ctrl;
    qDebug() << "initial velocity negative :" << QString("init state : %1,%2").arg(init_state.w).arg(init_state.dw)
             << QString::fromStdString(str(ctrl))
             << QString("final : %1").arg(final);
}

inline
void Trajectory1D::case21(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Control ctrl;
    double dw1 = sqrt((final*a_max)+(init_state.dw*init_state.dw/2.0));
    double t1 = (v_max-init_state.dw)/a_max;
    double t2 = (dw1-init_state.dw)/a_max;
    ctrl.control_case = ACCELERATION2;
    ctrl.effort = a_max;
    if(t1<t2)
    {
        ctrl.time = t1;
        ctrl.term.dw = v_max;
        ctrl.term.w = (v_max*v_max-init_state.dw*init_state.dw)/(2.0*a_max);
    }
    else
    {
        ctrl.time = t2;
        ctrl.term.dw = dw1;
        ctrl.term.w = (final/2.0) + (init_state.dw*init_state.dw/(2.0*a_max));
    }
    ctrl_seq.push_back(ctrl);
    qDebug() << "too slow or far away :"
             << QString("init state : %1").arg(str(init_state).data())
             << QString(str(ctrl).data())
             << QString("final : %1").arg(final)
             << QString("t1 : %1, t2 : %2").arg(t1).arg(t2)
             << QString("dw1 : %1").arg(dw1);
}

inline
void Trajectory1D::case22(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Control ctrl;
    ctrl.control_case = CRUISING;
    ctrl.effort = 0.0;
    ctrl.term.dw = v_max;
    ctrl.term.w = final - (v_max*v_max)/(2*a_max);
    ctrl.time = (final/v_max) - (v_max/(2*a_max));
    ctrl_seq.push_back(ctrl);
    qDebug() << "cruising :" << QString("init state : %1,%2").arg(init_state.w).arg(init_state.dw)
             << QString::fromStdString(str(ctrl))
             << QString("final : %1").arg(final);
}

inline
void Trajectory1D::case23(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Control ctrl;
    ctrl.control_case = DECELERATION1;
    ctrl.effort = -a_max;
    ctrl.term.dw = 0.0;
    ctrl.term.w = (init_state.dw*init_state.dw)/(2*a_max);
    ctrl.time = init_state.dw/a_max;
    ctrl_seq.push_back(ctrl);
    qDebug() << "braking :" << QString("init state : %1,%2").arg(init_state.w).arg(init_state.dw)
             << QString::fromStdString(str(ctrl))
             << QString("final : %1").arg(final);
}

inline
void Trajectory1D::case3(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Control ctrl;
    ctrl.control_case = DECELERATION2;
    ctrl.effort = -a_max;
    ctrl.term.dw = v_max;
    ctrl.term.w = (init_state.dw*init_state.dw-v_max*v_max)/(2*a_max);
    ctrl.time = (init_state.dw - v_max)/a_max;
    ctrl_seq.push_back(ctrl);
    qDebug() << "too fast :" << QString("init state : %1,%2").arg(init_state.w).arg(init_state.dw)
             << QString::fromStdString(str(ctrl))
             << QString("final : %1").arg(final);
}

inline
std::ostream& operator <<(std::ostream &out, const Trajectory1D::Control &ctrl)
{
    out << "[ effort : " << ctrl.effort << "; "
        << "time : " << ctrl.time << "; "
        << "teriminal state : [" << ctrl.term.w << "," << ctrl.term.dw << "]]";
    return out;
}

std::__cxx11::string Trajectory1D::str(Trajectory1D::Control &ctrl)
{
    std::stringstream out;
    out << "[ effort : " << ctrl.effort << "; "
        << "time : " << ctrl.time << "; "
        << "teriminal state : [" << ctrl.term.w << "," << ctrl.term.dw << "]]";
    return out.str();
}
