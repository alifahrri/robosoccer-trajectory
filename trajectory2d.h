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

class Controller
{
public:
    struct Control
    {
        Trajectory1D::Controller::Control x;
        Trajectory1D::Controller::Control y;
    };
    typedef std::pair<std::vector<State2D>,std::vector<double>> Trajectory;

public:
    Controller();
    void setLimit(double vmax, double amax);
    Control optimalControl(State2D initial_state, double xf, double yf);

private:
    double v_max;
    double a_max;
};

}


#endif // TRAJECTORY2D_H
