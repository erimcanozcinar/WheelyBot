#ifndef MATH_HPP
#define MATH_HPP

#include "Eigen/Dense"
#include "Eigen/Eigenvalues"

class mathOperations{
    private:
        /* #region: funcUnwrap variables */
        int k = 0;
        double deltaCorr = 0;
        double cumSumdeltaCorr = 0;
        double unwrapedOut = 0;
        /* #endregion */

    public:
        Eigen::Matrix3d RotateRoll(double thetha);
        Eigen::Matrix3d RotatePitch(double thetha);
        Eigen::Matrix3d RotateYaw(double thetha);
        Eigen::Vector3d quat2euler(double w, double x, double y, double z);
        Eigen::Matrix3d quat2RotMat(double qw, double qx, double qy, double qz);
        double Numdiff(double currX, double prevX, double dt);
        double Numdiff2nd(double currX, double prevX, double preprevX, double dt);
        double numIntegral(double In, double prevIn, double prevOut, double dt);
        double LowPassFilter(double Input, double prevOut, double freq, double dt);
        double funcUnwrap(double data, double prevData);
        Eigen::MatrixXd solveContinuousARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
};

#endif