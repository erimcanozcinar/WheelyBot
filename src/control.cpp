#include "control.hpp"

jointLevelControllers::jointLevelControllers()
{
    Kp.setZero(); Kd.setZero();
    Tau_pd.setZero();
    error.setZero();
    pre_error.setZero();
    derror.setZero();

    pre_ref_dqWheel.setZero();
    pre_ref_qWheel.setZero();
}

void jointLevelControllers::jointLevelPD(Eigen::Vector4d ref_qJoint, Eigen::Vector4d measured_qJoint, double dT)
{

    Kp << 50, 0, 0, 0,
          0, 50, 0, 0,
          0, 0, 50, 0,
          0, 0, 0, 50;

    Kd << 5, 0, 0, 0,
          0, 5, 0, 0,
          0, 0, 5, 0,
          0, 0, 0, 5;


    error = ref_qJoint - measured_qJoint;
    for(int i=0; i<4; i++)
    {
        derror(i) = mathOp.Numdiff(error(i), pre_error(i), dT);
    }
    pre_error = error;

    Tau_pd = Kp*error + Kd*derror;
}

void jointLevelControllers::wheelsPI(Eigen::Vector2d ref_dqWheel, Eigen::Vector2d qWheel, Eigen::Vector2d dqWheel, double dT)
{   

    for(int i=0; i<2; i++)
    {
        ref_qWheel(i) = mathOp.numIntegral(ref_dqWheel(i), pre_ref_dqWheel(i), pre_ref_qWheel(i,1), dT);
    }
    pre_ref_dqWheel = ref_dqWheel;
    pre_ref_qWheel = ref_qWheel;

    tauWheel_L = 0.1*(ref_dqWheel(0) - dqWheel(0)) + 0.001*(ref_qWheel(0) - qWheel(0));
    tauWheel_R = 0.1*(ref_dqWheel(1) - dqWheel(1)) + 0.001*(ref_qWheel(1) - qWheel(1));
}




stabilizationController::stabilizationController()
{
    Pitch = 0.0;
    pre_Pitch = 0.0;
    dPitch = 0.0;
    des_dqWheels = 0.0;
}

void stabilizationController::torsoStabilization(double ref_Pitch, Eigen::Vector3d torsoEulerAngles, double dt)
{
    Pitch = torsoEulerAngles(1);
    dPitch = mathOp.Numdiff(Pitch, pre_Pitch, dt);
    pre_Pitch = Pitch;

    ref_dPitch = mathOp.Numdiff(ref_Pitch, pre_ref_Pitch, dt);
    pre_ref_Pitch = ref_Pitch; 

    des_dqWheels = -6000*(ref_Pitch - Pitch) - 600*(ref_dPitch - dPitch);
    std::cout << des_dqWheels << std::endl;

}




torsoVelControllers::torsoVelControllers()
{
    errorV = 0.0; pre_errorV = 0.0; derrorV = 0.0;
    errorW = 0.0; pre_errorW = 0.0; derrorW = 0.0;
}

void torsoVelControllers::linearVelController(double ref_Vtorso, double meas_Vtorso, double dT)
{
    errorV = ref_Vtorso - meas_Vtorso;
    derrorV = mathOp.Numdiff(errorV, pre_errorV, dT);
    pre_errorV = errorV;
    ref_pitch = 0.00*errorV + 0*derrorV;
}

void torsoVelControllers::angularVelController(double ref_Wtorso, double meas_Wtorso, double dT)
{
    errorW = ref_Wtorso - meas_Wtorso;
    derrorW = mathOp.Numdiff(errorW, pre_errorW, dT);
    pre_errorW = errorW;
    ref_wheel_angVel = 10*errorW + 1*derrorW;
}