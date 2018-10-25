#include "trajectoryplanner.h"

RobotTrajectory::Generator::Generator()
{

}

void RobotTrajectory::Generator::setLimit(double v_lin, double a_lin, double v_ang, double a_ang)
{
    linear.setLimit(v_lin,a_lin);
    angular.setLimit(v_ang,a_ang);
}

void RobotTrajectory::Generator::generate(RobotTrajectory::State s, RobotTrajectory::State final_pos, double *t_lin, double *t_ang)
{
    auto tf_angular(0.0);
    auto tf_linear(0.0);
    angular_control = angular.optimalControl({s.w,s.dw},final_pos.w,tf_angular);
    linear_control = linear.optimalControl({{s.x,s.dx},{s.y,s.dy}},final_pos.x,final_pos.y);
    if(t_lin)
        *t_lin = linear_control.x.final_time;
    if(t_ang)
        *t_ang = angular_control.final_time;
}

RobotTrajectory::Trajectory RobotTrajectory::Generator::getTrajectory(double t0, double tf, double dt)
{
    Trajectory trajectory;
    auto w = Trajectory1D::Controller::getTrajectory(angular_control,t0,tf,dt);
    auto x = Trajectory1D::Controller::getTrajectory(linear_control.x,t0,tf,dt);
    auto y = Trajectory1D::Controller::getTrajectory(linear_control.y,t0,tf,dt);
    trajectory.second = (w.second.size()>x.second.size()) ? w.second : x.second;
    trajectory.second = (trajectory.second.size()>y.second.size()) ? trajectory.second : y.second;
    fillTrajectory(w,trajectory.second.size()-w.second.size());
    fillTrajectory(x,trajectory.second.size()-x.second.size());
    fillTrajectory(y,trajectory.second.size()-y.second.size());
    for(size_t i=0; i<trajectory.second.size(); i++)
    {
        trajectory.first.push_back({x.first.at(i).w,y.first.at(i).w,w.first.at(i).w,
                                    x.first.at(i).dw,y.first.at(i).dw,w.first.at(i).dw});
    }
    return trajectory;
}

RobotTrajectory::State RobotTrajectory::Generator::getControl(double dt)
{
  State ret;
  auto x = Trajectory1D::Controller::getState(linear_control.x,dt);
  auto y = Trajectory1D::Controller::getState(linear_control.y,dt);
  auto w = Trajectory1D::Controller::getState(angular_control,dt);
  ret.x = x.w;
  ret.dx = x.dw;
  ret.y = y.w;
  ret.dy = y.dw;
  ret.w = w.w;
  ret.dw = w.dw;
  return ret;
}

inline
void RobotTrajectory::Generator::fillTrajectory(Trajectory1D::Controller::Trajectory &to_fill, std::size_t size)
{
    auto s = to_fill.first.back();
    for(size_t i=0; i<size; i++)
        to_fill.first.push_back(s);
}
