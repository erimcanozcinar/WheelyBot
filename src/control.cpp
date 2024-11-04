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

    Kp << 100, 0, 0, 0,
          0, 100, 0, 0,
          0, 0, 100, 0,
          0, 0, 0, 100;

    Kd << 10, 0, 0, 0,
          0, 10, 0, 0,
          0, 0, 10, 0,
          0, 0, 0, 10;


    error = ref_qJoint - measured_qJoint;
    for(int i=0; i<4; i++)
    {
        derror(i) = Numdiff(error(i), pre_error(i), dT);
    }
    pre_error = error;

    Tau_pd = Kp*error + Kd*derror;
}

void jointLevelControllers::wheelsPI(Eigen::Vector2d ref_dqWheel, Eigen::Vector2d qWheel, Eigen::Vector2d dqWheel, double dT)
{   

    for(int i=0; i<2; i++)
    {
        ref_qWheel(i) = numIntegral(ref_dqWheel(i), pre_ref_dqWheel(i), pre_ref_qWheel(i,1), dT);
    }
    pre_ref_dqWheel = ref_dqWheel;
    pre_ref_qWheel = ref_qWheel;

    tauWheel_L = 0.01*(ref_dqWheel(0) - dqWheel(0)) + 0.001*(ref_qWheel(0) - qWheel(0));
    tauWheel_R = 0.01*(ref_dqWheel(1) - dqWheel(1)) + 0.001*(ref_qWheel(1) - qWheel(1));
}




modelBasedControllers::modelBasedControllers()
{
    lqrStates.setZero(); lqrStates_des.setZero();
    // lqrGains << 7.0486, 2.2361, 2.2361, 1.8536, 3.3106,  2.3876,
    //             7.0486, 2.2361,-2.2361, 1.8536, 3.3106, -2.3876;
    // lqrGains << -90.8480, -31.6228,  2.2361, -5.8546, -51.2304,  2.3876,
    //             -90.8480, -31.6228, -2.2361, -5.8546, -51.2304, -2.3876;
    // lqrGains << -62.1112, -15.8114, -7.0711, -13.0206, -14.3455, -3.1931,
    //             -62.1112, -15.8114,  7.0711, -13.0206, -14.3455,  3.1931;

    lqrGains << -6.7142, -1.5811, -0.7071, -1.9659, -1.5496, -0.1867,
                -6.7142, -1.5811,  0.7071, -1.9659, -1.5496,  0.1867;

    matA.resize(6,6); matB.resize(6,2); matQ.resize(6,6); matR.resize(2,2); 

    matA << 0.0000,         0.0000,         0.0000,         1.0000,         0.0000,         0.0000,
            0.0000,         0.0000,         0.0000,         0.0000,         1.0000,         0.0000,
            0.0000,         0.0000,         0.0000,         0.0000,         0.0000,         1.0000,
            36.2822,        0.0000,         0.0000,         0.0000,         0.0000,         0.0000,
            -5.6140,        0.0000,         0.0000,         0.0000,         0.0000,         0.0000,
            0.0000,         0.0000,         0.0000,         0.0000,         0.0000,         0.0000;

    matB << 0.0000,         0.0000,
            0.0000,         0.0000,
            0.0000,         0.0000,
            -3.0106,        -3.0106,
            1.2798,         1.2798,
            -0.9188,        0.9188;

    matQ << 500,     0,     0,     0,     0,     0,
            0,     1,     0,     0,     0,     0,
            0,     0,     1,     0,     0,     0,
            0,     0,     0,     50,     0,     0,
            0,     0,     0,     0,     1,     0,
            0,     0,     0,     0,     0,     1;
    
    matR << 0.1, 0, 
            0, 0.1;


}

void modelBasedControllers::LQR()
{    
    // matP = solveContinuousARE(matA, matB, matQ, matR);
    // std::cout << matR.inverse()*matB.transpose()*matP << std::endl;
    // std::cout << "-----------------------------------------------------------------------------" << std::endl;
    // lqrGains = matR.inverse()*matB.transpose()*matP;
    lqrTau = 1e3*lqrGains*(lqrStates_des - lqrStates);
}

