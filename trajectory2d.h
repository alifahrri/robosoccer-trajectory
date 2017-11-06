#ifndef TRAJECTORY2D_H
#define TRAJECTORY2D_H

#include <vector>
#include "trajectory1d.h"

namespace Trajectory2D {

struct State2D {
    Trajectory1D::State x;
    Trajectory1D::State y;
};

struct Control2D {
    Trajectory1D::Control x_ctrl;
    Trajectory1D::Control y_ctrl;
};

class OptimalController
{
public:
    struct Control
    {
        Trajectory1D::OptimalController::ControlSequence x;
        Trajectory1D::OptimalController::ControlSequence y;
    };
    typedef std::pair<std::vector<State2D>,double> Trajectory;

public:
    OptimalController();
    void setLimit(double vmax, double amax);
    Control optimalControl(State2D initial_state, double xf, double yf);

private:
    double v_max;
    double a_max;
};

}


#endif // TRAJECTORY2D_H
