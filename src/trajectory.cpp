#include "trajectory.hpp"

void trajectoryGeneration::FuncPoly3rd(double RealTime, double t_start, double t_end, double Z0, double dZ0, double Ze, double dZe)
{
    tw = t_end - t_start;
    Rt = RealTime - t_start;

    n0 = Z0;
    n1 = dZ0;
    n2 = (3 * Ze) / pow(tw, 2) - (3 * Z0) / pow(tw, 2) - (2 * dZ0) / tw - dZe / tw;
    n3 = (2 * Z0) / pow(tw, 3) - (2 * Ze) / pow(tw, 3) + dZ0 / pow(tw, 2) + dZe / pow(tw, 2);

    Pos = n0 + n1 * Rt + n2 * pow(Rt, 2) + n3 * pow(Rt, 3);
    Vel = n1 + 2 * n2 * Rt + 3 * n3 * pow(Rt, 2);

    if (RealTime >= t_start && RealTime <= t_end)
    {
        Pos = n0 + n1 * Rt + n2 * pow(Rt, 2) + n3 * pow(Rt, 3);
        Vel = n1 + 2 * n2 * Rt + 3 * n3 * pow(Rt, 2);
    }
    else if (RealTime < t_start)
    {
        Pos = Z0;
        Vel = dZ0;
    }
    else if (RealTime > t_end)
    {
        Pos = Ze;
        Vel = dZe;
    }
    trajOut << Pos, Vel;
}