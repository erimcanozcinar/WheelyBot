#ifndef MATH_HPP
#define MATH_HPP

#include "Eigen/Dense"

class mathOperations{

    public:
        Eigen::Matrix3d RotateRoll(double thetha);
        Eigen::Matrix3d RotatePitch(double thetha);
        Eigen::Matrix3d RotateYaw(double thetha);
        Eigen::Vector3d quat2euler(double w, double x, double y, double z);
        Eigen::Matrix3d quat2RotMat(double qw, double qx, double qy, double qz);
        double Numdiff(double currX, double prevX, double dt);
        double numIntegral(double In, double prevIn, double prevOut, double dt);
};

#endif