#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include "trajectory1d.h"
#include "trajectory2d.h"

namespace RobotTrajectory {

using namespace std;

struct State
{
    double x;
    double y;
    double w;
    double dx;
    double dy;
    double dw;
};

typedef pair<vector<State>,vector<double>> Trajectory;

class Generator
{
public:
    Generator();
    void setLimit(double v_lin, double a_lin, double v_ang, double a_ang);
    void generate(State s, State final_pos, double *t_lin=nullptr, double *t_ang=nullptr);
    Trajectory getTrajectory(double t0, double tf, double dt);
    State getControl(double dt);
private:
    void fillTrajectory(Trajectory1D::Controller::Trajectory &to_fill, size_t size);
private:
    Trajectory1D::Controller angular;
    Trajectory2D::Controller linear;
    Trajectory1D::Controller::Control angular_control;
    Trajectory2D::Controller::Control linear_control;
};

}

#endif // TRAJECTORYPLANNER_H
