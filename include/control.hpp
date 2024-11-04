#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "iostream"
#include "Eigen/Dense"
#include "math.hpp"

class jointLevelControllers : virtual protected mathOperations{
    private:
        Eigen::Matrix4d Kp, Kd;
        Eigen::Vector4d error, pre_error, derror;

        Eigen::Vector2d ref_qWheel, pre_ref_qWheel, pre_ref_dqWheel;

    public:
        jointLevelControllers();

        Eigen::Vector4d Tau_pd;
        void jointLevelPD(Eigen::Vector4d ref_qJoint, Eigen::Vector4d measured_qJoint, double dT);

        double tauWheel_L, tauWheel_R;
        void wheelsPI(Eigen::Vector2d ref_dqWheel, Eigen::Vector2d qWheel, Eigen::Vector2d dqWheel, double dT);

};

class modelBasedControllers : virtual protected mathOperations{
    private:
        Eigen::MatrixXd lqrGains{2,6};
    public:
        modelBasedControllers();

        Eigen::MatrixXd matA, matB, matQ, matR, matP;
        Eigen::Vector2d lqrTau;
        Eigen::VectorXd lqrStates{6}, lqrStates_des{6};
        void LQR();
};

#endif