#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "iostream"
#include "Eigen/Dense"
#include "math.hpp"

class jointLevelControllers{
    private:
        mathOperations mathOp;


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

class stabilizationController{
    private:
        mathOperations mathOp;

        double Pitch, pre_Pitch, dPitch;
        double pre_ref_Pitch, ref_dPitch;
    public:
        stabilizationController();

        double des_dqWheels;
        void torsoStabilization(double ref_Pitch, Eigen::Vector3d torsoEulerAngles, double dt);
};

class torsoVelControllers{
    private:
        mathOperations mathOp;

        double errorV, pre_errorV, derrorV;
        double errorW, pre_errorW, derrorW;

    protected:
    public:
        torsoVelControllers();

        double ref_pitch;
        void linearVelController(double ref_Vtorso, double meas_Vtorso, double dT);

        double ref_wheel_angVel;
        void angularVelController(double ref_Wtorso, double meas_Wtorso, double dT);
};

#endif