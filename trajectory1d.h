#ifndef TRAJECTORY1D_H
#define TRAJECTORY1D_H

#include <vector>

class Trajectory1D
{
public:
    enum Case {
        INITIAL_STATE,
        ACCELERATION1,  //case 1
        ACCELERATION2,  //case 2.1
        CRUISING,       //case 2.2
        DECELERATION1,  //case 2.3
        DECELERATION2,  //case 3
    };
    struct State {
        double w;
        double dw;
    };
    struct Control {
        Case control_case;
        double effort;
        double time;
        State term;
    };

    struct ControlSequence : std::vector<Control>
    {
        double final_time;
        double offset;
        double distance;
    };

public:
    Trajectory1D();
    void setLimit(double vmax=1.0, double amax=1.0);
    ControlSequence optimalControl(State init_state, double final_state, double& final_time);

    static State getState(const ControlSequence &ctrl, double time);

private:
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

#endif // TRAJECTORY1D_H
