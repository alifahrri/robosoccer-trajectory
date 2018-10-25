#include "trajectory1d.h"
#include <cmath>
#include <iostream>
#include <QDebug>
#include <stdexcept>
#include <sstream>
#define sign(X) (X<0?-1.0:1.0)

using namespace Trajectory1D;

Controller::Controller()
    : a_max(1.0),
      v_max(1.0)
{

}

void Controller::setLimit(double vmax, double amax)
{
    v_max = vmax;
    a_max = amax;
}

Controller::Control Controller::optimalControl(Trajectory1D::State init_state, double final_state, double &final_time)
{
    Control ctrl_seq;
    State initial_state = init_state;
    Trajectory1D::Control ctrl;
    ctrl.term = init_state;
    ctrl.time = 0.0;
    ctrl.effort = 0.0;
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
    ctrl_seq.a_max = a_max;
    ctrl_seq.v_max = v_max;
    return ctrl_seq;
}

Trajectory1D::State Controller::getState(const Controller::Control &ctrl, double time)
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

Controller::Trajectory Controller::getTrajectory(const Controller::Control &ctrl, double t0, double tf, double dt)
{
    Trajectory trajectory;
    int t_count = (int)((tf-t0)/dt);
    int idx(0);
    double time(0.0);
    double time_init(0.0);
    auto s0 = ctrl.at(0).term;
    for(size_t i=1; i<t_count; i++)
    {
        double now = (double)i*dt;
        if(now>time)
        {
            idx++;
            if(idx>=(ctrl.size()))
                break;
            time_init = time;
            time += ctrl.at(idx).time;
        }
        const auto& c = ctrl.at(idx);
        auto vel = s0.dw + c.effort*dt;
        auto pos = s0.w + vel*dt;
        s0.w = pos;
        s0.dw = vel;
        trajectory.first.push_back({pos,vel});
        trajectory.second.push_back(t0 + now);
    }
    if(trajectory.first.empty())
    {
        auto c = ctrl.at(0);
        trajectory.first.push_back(c.term);
        trajectory.second.push_back(t0);
    }
    return trajectory;
}

double Controller::setMaxEffort(Controller::Control &ctrl, double amax)
{
    auto final_time(0.0);
    auto s0 = ctrl.at(0).term;
    for(auto& c : ctrl)
    {
        switch(c.control_case)
        {
        case CRUISING :
        {
            c.time += (ctrl.v_max/ctrl.a_max) - (ctrl.v_max/amax);
            c.effort = 0.0;
            auto vm_sq = ctrl.v_max*ctrl.v_max;
            c.term.w += -(vm_sq/(2*ctrl.a_max)) + (vm_sq/(2*amax));
        }
            break;
        case ACCELERATION2_1 :
        {
            c.time = c.time*ctrl.a_max/amax;
            c.effort = c.effort*amax/fabs(c.effort);
            c.term.w *= (ctrl.a_max/amax);
        }
            break;
        case ACCELERATION2_2 :
        {
            c.time = c.time*ctrl.a_max/amax;
            c.effort = c.effort*amax/fabs(c.effort);
            auto v0_sq = s0.dw*s0.dw;
            auto dw1_sq = c.term.dw*c.term.dw;
            auto dw1 = dw1_sq-v0_sq/2.0;
            dw1 *= amax/ctrl.a_max;
            dw1 += v0_sq/2.0;
            dw1 = sqrt(dw1);
            c.term.w += -(v0_sq/(2*ctrl.a_max)) + (v0_sq/(2*amax));
            c.term.dw = dw1;
        }
            break;
        case ACCELERATION1 :
        case DECELERATION1 :
        case DECELERATION2 :
            c.time = c.time*ctrl.a_max/amax;
            c.effort = c.effort*amax/fabs(c.effort);
            c.term.w *= (ctrl.a_max/amax);
            break;
        default :
            break;
        }
        final_time += c.time;
        s0 = c.term;
    }
    ctrl.final_time = final_time;
    ctrl.a_max = amax;
    return final_time;
}

std::__cxx11::string Controller::str(Trajectory1D::State &s)
{
    std::stringstream str;
    str << "[" << s.w << "," << s.dw << "]";
    return str.str();
}

inline
void Controller::applyControl(Controller::Control &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    double wf = final - init_state.w;
    double normal_wf = wf*sign(wf);
    double w0 = 0.0;
    double dw0 = init_state.dw*sign(wf); //normalized dw0
    double tmp = dw0*dw0/(2.0*a_max);
    State initial_state = {w0,dw0};
//    qDebug() << "apply control :" << "final :" << final
//             << "wf :" << wf
//             << "normal wf :" << normal_wf
//             << "tmp :" << tmp
//             << "init_state :" << str(init_state).data()
//             << "normalwf == tmp?" << (normal_wf==tmp?"true;" : "false;")
//             << "normalwf > tmp?" << (normal_wf>tmp?"true;" : "false;")
//             << "normalwf < tmp?" << (normal_wf<tmp?"true;" : "false;");
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
void Controller::case1(Controller::Control &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Trajectory1D::Control ctrl;
    ctrl.control_case = ACCELERATION1;
    ctrl.effort = a_max;
    ctrl.time = -init_state.dw/a_max;
    ctrl.term.w = -init_state.dw*init_state.dw/(2.0*a_max);
    ctrl.term.dw = 0;
    ctrl_seq.push_back(ctrl);
    std::ostringstream ctrl_str;
    ctrl_str << ctrl;
//    qDebug() << "initial velocity negative :" << QString("init state : %1,%2").arg(init_state.w).arg(init_state.dw)
//             << QString::fromStdString(str(ctrl))
//             << QString("final : %1").arg(final);
}

inline
void Controller::case21(Controller::Control &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Trajectory1D::Control ctrl;
    double dw1 = sqrt((final*a_max)+(init_state.dw*init_state.dw/2.0));
    double t1 = (v_max-init_state.dw)/a_max;
    double t2 = (dw1-init_state.dw)/a_max;
    ctrl.effort = a_max;
    if(t1<t2)
    {
        ctrl.control_case = ACCELERATION2_1;
        ctrl.time = t1;
        ctrl.term.dw = v_max;
        ctrl.term.w = (v_max*v_max-init_state.dw*init_state.dw)/(2.0*a_max);
    }
    else
    {
        ctrl.control_case = ACCELERATION2_2;
        ctrl.time = t2;
        ctrl.term.dw = dw1;
        ctrl.term.w = (final/2.0) + (init_state.dw*init_state.dw/(2.0*a_max));
    }
    ctrl_seq.push_back(ctrl);
//    qDebug() << "too slow or far away :"
//             << QString("init state : %1").arg(str(init_state).data())
//             << QString(str(ctrl).data())
//             << QString("final : %1").arg(final)
//             << QString("t1 : %1, t2 : %2").arg(t1).arg(t2)
//             << QString("dw1 : %1").arg(dw1);
}

inline
void Controller::case22(Controller::Control &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Trajectory1D::Control ctrl;
    ctrl.control_case = CRUISING;
    ctrl.effort = 0.0;
    ctrl.term.dw = v_max;
    ctrl.term.w = final - (v_max*v_max)/(2*a_max);
    ctrl.time = (final/v_max) - (v_max/(2*a_max));
    ctrl_seq.push_back(ctrl);
//    qDebug() << "cruising :" << QString("init state : %1,%2").arg(init_state.w).arg(init_state.dw)
//             << QString::fromStdString(str(ctrl))
//             << QString("final : %1").arg(final);
}

inline
void Controller::case23(Controller::Control &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Trajectory1D::Control ctrl;
    ctrl.control_case = DECELERATION1;
    ctrl.effort = -a_max;
    ctrl.term.dw = 0.0;
    ctrl.term.w = (init_state.dw*init_state.dw)/(2*a_max);
    ctrl.time = init_state.dw/a_max;
    ctrl_seq.push_back(ctrl);
//    qDebug() << "braking :" << QString("init state : %1,%2").arg(init_state.w).arg(init_state.dw)
//             << QString::fromStdString(str(ctrl))
//             << QString("final : %1").arg(final);
}

inline
void Controller::case3(Controller::Control &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Trajectory1D::Control ctrl;
    ctrl.control_case = DECELERATION2;
    ctrl.effort = -a_max;
    ctrl.term.dw = v_max;
    ctrl.term.w = (init_state.dw*init_state.dw-v_max*v_max)/(2*a_max);
    ctrl.time = (init_state.dw - v_max)/a_max;
    ctrl_seq.push_back(ctrl);
//    qDebug() << "too fast :" << QString("init state : %1,%2").arg(init_state.w).arg(init_state.dw)
//             << QString::fromStdString(str(ctrl))
//             << QString("final : %1").arg(final);
}

inline
std::ostream& operator <<(std::ostream &out, const Trajectory1D::Control &ctrl)
{
    out << "[ effort : " << ctrl.effort << "; "
        << "time : " << ctrl.time << "; "
        << "teriminal state : [" << ctrl.term.w << "," << ctrl.term.dw << "]]";
    return out;
}

std::__cxx11::string Controller::str(Trajectory1D::Control &ctrl)
{
    std::stringstream out;
    out << "[ effort : " << ctrl.effort << "; "
        << "time : " << ctrl.time << "; "
        << "teriminal state : [" << ctrl.term.w << "," << ctrl.term.dw << "]]";
    return out.str();
}
