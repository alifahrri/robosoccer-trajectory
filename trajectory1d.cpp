#include "trajectory1d.h"
#include <cmath>
#define sign(X) (X<0?-1:1)

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

Trajectory1D::ControlSequence Trajectory1D::optimalControl(Trajectory1D::State init_state, double final_state)
{
    ControlSequence ctrl_seq;
    State initial_state = init_state;
initial:
    double t = 0.0;
    double wf = final_state;
    initial_state.w = 0.0;
    initial_state.dw *= sign(init_state.w);
    wf *= sign(init_state.w);
check_case :
    double dw0 = initial_state.dw;
    double tmp = dw0*dw0/(2*a_max);
    if(dw0<0)
        case1(ctrl_seq,initial_state,wf);
    else if(dw0<v_max && wf>tmp)
        case21(ctrl_seq,initial_state,wf);
    else if(dw0==v_max && wf>tmp)
        case22(ctrl_seq,initial_state,wf);
    else if(dw0<=v_max && wf<=tmp)
        case23(ctrl_seq,initial_state,wf);
    else if(dw0>v_max)
        case3(ctrl_seq,initial_state,wf);
    t += ctrl_seq.back().time;
    wf -= ctrl_seq.back().term.w;
    initial_state.dw = ctrl_seq.back().term.dw;
    initial_state.dw *= sign(wf);
    wf *= sign(wf);
    if((ctrl_seq.back().term.dw != 0.0) || (ctrl_seq.back().control_case == ACCELERATION1))
        goto check_case;
    return ctrl_seq;
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
