#include "trajectory1d.h"
#include <cmath>
#include <iostream>
#define sign(X) (X<0?-1:1)

Trajectory1D::Trajectory1D()
    : a_max(1.0),
      v_max(1.0)
{
//    std::cout << "-0.0 == 0.0 ? " << ((-0.0 == 0.0) ? "true\n" : "false\n");
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
    double distance = 0.0;
    double t = 0.0;
    double wf = final_state + ctrl_seq.offset;
    double dw0, tmp;
    initial_state.w = 0.0;
normalize:
    double w_diff = (final_state + ctrl_seq.offset) - ctrl_seq.distance;
    if(w_diff == 0.0)
        goto done;
    initial_state.dw *= sign(wf);
    wf *= sign(wf);
check_case :
    dw0 = initial_state.dw;
    tmp = dw0*dw0/(2*a_max);
    if(dw0<0)
        case1(ctrl_seq,initial_state,wf);
    else if(dw0<v_max && dw0>=0.0 && wf>tmp)
        case21(ctrl_seq,initial_state,wf);
    else if(dw0==v_max && wf>tmp)
        case22(ctrl_seq,initial_state,wf);
    else if(dw0<=v_max && dw0>0.0 && wf<=tmp)
        case23(ctrl_seq,initial_state,wf);
    else if(dw0>v_max)
        case3(ctrl_seq,initial_state,wf);
    else
    {
        std::cout << "error!!!\n";
        exit(-1);
    }

//    t += ctrl_seq.back().time;
//    initial_state.dw = ctrl_seq.back().term.dw;
//    initial_state.dw *= sign(wf);
//    ctrl_seq.distance += ctrl_seq.back().term.w;
//    wf -= ctrl_seq.back().term.w;
//    wf *= sign(wf);

    t += fabs(ctrl_seq.back().time);
    ctrl_seq.back().effort *= sign(w_diff);
    ctrl_seq.back().term.dw *= sign(w_diff);
    initial_state.dw = ctrl_seq.back().term.dw;
    ctrl_seq.distance += ctrl_seq.back().term.w;
    wf -= ctrl_seq.back().term.w;
    wf *= sign(w_diff);
    ctrl_seq.back().term.w *= sign(w_diff);
    distance = ctrl_seq.back().term.w;

    if((ctrl_seq.back().term.dw != 0.0) || (fabs(wf)>0.0))
        goto normalize;
done:
    final_time = t;
    ctrl_seq.final_time = t;
    return ctrl_seq;
}

Trajectory1D::State Trajectory1D::getState(const Trajectory1D::ControlSequence &ctrl, double time)
{
    State s;
    State s0 = {0.0,0.0};
    double t = 0.0;
    double dt = time;
    for(int i=1; i<ctrl.size(); ++i)
    {
        auto c = ctrl.at(i);
        t += c.time;
        auto s_1 = ctrl.at(i-1).term; // prev state
        s0.w += s_1.w;
        s0.dw = s_1.dw;
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

inline
void Trajectory1D::applyControl(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    double offset = (init_state.w != 0.0) ? -init_state.w : 0.0;
    double wf = (final + offset);
    double normal_wf = wf*sign(wf);
    double w0 = 0.0;
    double dw0 = init_state.dw*sign(wf);
    double tmp = dw0*dw0/(2*a_max);
    if(dw0<0)
        case1(ctrl_seq,initial_state,normal_wf);
    else if(dw0<v_max && dw0>=0.0 && normal_wf>tmp)
        case21(ctrl_seq,initial_state,normal_wf);
    else if(dw0==v_max && normal_wf>tmp)
        case22(ctrl_seq,initial_state,normal_wf);
    else if(dw0<=v_max && dw0>0.0 && normal_wf<=tmp)
        case23(ctrl_seq,initial_state,normal_wf);
    else if(dw0>v_max)
        case3(ctrl_seq,initial_state,normal_wf);
    else
    {
        std::cout << "error!!!\n";
        exit(-1);
    }
    ctrl_seq.back().effort *= sign(wf);
    ctrl_seq.back().term.dw *= sign(wf);
    ctrl_seq.back().term.w *= sign(wf);
}

inline
void Trajectory1D::case1(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Control ctrl;
    ctrl.control_case = ACCELERATION1;
    ctrl.effort = a_max;
    ctrl.time = -init_state.dw/a_max;
    ctrl.term.w = -init_state.dw*init_state.dw/(2*a_max);
    ctrl.term.dw = 0;
    ctrl_seq.push_back(ctrl);
}

inline
void Trajectory1D::case21(Trajectory1D::ControlSequence &ctrl_seq, Trajectory1D::State &init_state, double final)
{
    Control ctrl;
    double dw1 = sqrt(final*a_max+(init_state.dw*init_state.dw/2));
    double t1 = (v_max-init_state.dw)/a_max;
    double t2 = (dw1-init_state.dw)/a_max;
    ctrl.control_case = ACCELERATION2;
    ctrl.effort = a_max;
    if(t1<t2)
    {
        ctrl.time = t1;
        ctrl.term.dw = v_max;
        ctrl.term.w = (v_max*v_max-init_state.dw*init_state.dw)/(2*a_max);
    }
    else
    {
        ctrl.time = t2;
        ctrl.term.w = final/2 + init_state.dw*init_state.dw/(2*a_max);
        ctrl.term.dw = dw1;
    }
    ctrl_seq.push_back(ctrl);
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
}
