#ifndef TRAJECTORY1D_H
#define TRAJECTORY1D_H

#include <vector>
#include <iostream>

namespace Trajectory1D {

struct State {
    double w;
    double dw;
};

enum Case {
    INITIAL_STATE,
    ACCELERATION1,  //case 1
    ACCELERATION2_1,  //case 2.1 subcase 1
    ACCELERATION2_2,  //case 2.1 subcase 1
    CRUISING,       //case 2.2
    DECELERATION1,  //case 2.3
    DECELERATION2,  //case 3
};

struct Control {
    Case control_case;
    double effort;
    double time;
    State term;
};

class OptimalController
{
public:

    struct ControlSequence : std::vector<Control>
    {
        double final_time;
        double time_offset;
        double offset;
        double distance;
        double a_max;
        double v_max;
    };

    typedef std::pair<std::vector<State>,std::vector<double>> Trajectory;

public:
    OptimalController();
    void setLimit(double vmax=1.0, double amax=1.0);
    ControlSequence optimalControl(State init_state, double final_state, double& final_time);

    static State getState(const ControlSequence &ctrl, double time);
    static Trajectory getTrajectory(const ControlSequence& ctrl, double t0, double tf, double dt);
    static double setMaxEffort(ControlSequence &ctrl, double amax);

private:
    std::string str(State &s);
    std::string str(Control &ctrl);
    void applyControl(ControlSequence& ctrl_seq, State& init_state, double final);
    void case1(ControlSequence& ctrl_seq, State& init_state, double final);
    void case21(ControlSequence& ctrl_seq, State& init_state, double final);
    void case22(ControlSequence& ctrl_seq, State& init_state, double final);
    void case23(ControlSequence& ctrl_seq, State& init_state, double final);
    void case3(ControlSequence& ctrl_seq, State& init_state, double final);
private:
    double v_max;
    double a_max;
};

}

std::ostream& operator << (std::ostream& out, const Trajectory1D::Control &ctrl);

#endif // TRAJECTORY1D_H
