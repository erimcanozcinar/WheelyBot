#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "iostream"
#include "Eigen/Dense"

class trajectoryGeneration {
    private:
        double tw = 0.0;
        double Rt = 0.0;
        double Pos = 0.0;
        double Vel = 0.0;

        double n0 = 0.0;
        double n1 = 0.0;
        double n2 = 0.0;
        double n3 = 0.0;

    public:
        Eigen::Vector2d trajOut;
        void FuncPoly3rd(double RealTime, double t_start, double t_end, double Z0, double dZ0, double Ze, double dZe);
};

#endif