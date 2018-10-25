#include "trajectory2d.h"
#include "trajectory1d.h"
#include <cmath>
#include <QDebug>

#define tan_deg(x) (tan(x*M_PI/180))
#define sin_deg(x) (sin(x*M_PI/180))
#define cos_deg(x) (cos(x*M_PI/180))

using namespace Trajectory2D;

Controller::Controller()
    : v_max(1.0),
      a_max(1.0)
{

}

void Controller::setLimit(double vmax, double amax)
{
    a_max = sqrt(amax);
    v_max = sqrt(vmax);
}

Controller::Control Controller::optimalControl(Trajectory2D::State2D initial_state, double xf, double yf)
{
    double alpha = 45.0;
    double alpha_min(0.0);
    double alpha_max(90.0);
    double ax_max(1.0);
    double ay_max(1.0);
    double vx_max(1.0);
    double vy_max(1.0);

    Trajectory1D::Controller x_trajectory;
    Trajectory1D::Controller y_trajectory;

    auto tx(0.0);
    auto ty(0.0);

find_minimum_time:

    ax_max = a_max*cos_deg(alpha);
    ay_max = a_max*sin_deg(alpha);
    vx_max = v_max*cos_deg(alpha);
    vy_max = v_max*sin_deg(alpha);

    x_trajectory.setLimit(vx_max,ax_max);
    y_trajectory.setLimit(vy_max,ay_max);
    auto x_ctrl = x_trajectory.optimalControl(initial_state.x,xf,tx);
    auto y_ctrl = y_trajectory.optimalControl(initial_state.y,yf,ty);

    auto dt = fabs(tx-ty);
    if(dt<0.01)
        goto done;
    else if(tx==0.0)
    {
        y_trajectory.setLimit(v_max,a_max);
        y_ctrl = y_trajectory.optimalControl(initial_state.y,yf,ty);;
        goto done;
    }
    else if(ty==0.0)
    {
        x_trajectory.setLimit(v_max,a_max);
        x_ctrl = x_trajectory.optimalControl(initial_state.x,xf,tx);;
        goto done;
    }

    if(tx>ty)
    {
        alpha_max = alpha;
        alpha = alpha_max-(alpha-alpha_min)/2.0;
    }
    else
    {
        alpha_min = alpha;
        alpha = (alpha_max-alpha)/2.0+alpha_min;
    }

    qDebug() << dt << alpha << alpha_min << alpha_max;
    goto find_minimum_time;
done:
    qDebug() << alpha << ax_max << ay_max << vx_max << vy_max;
    return Control({x_ctrl,y_ctrl});
}
