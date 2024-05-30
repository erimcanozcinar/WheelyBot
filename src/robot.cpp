#include "robot.hpp"

mathOperations mathOp;
jointLevelControllers jointController;

robotLeg::robotLeg(char leg)
{
    switch (leg)
    {
    case 'l':
    case 'L':
        legIndex = 1;
        break;
    case 'r':
    case 'R':
        legIndex = -1;
        break;
    }
}

void robotLeg::solveLegFK(Eigen::Matrix3d rootOrient, Eigen::Vector2d jointAngles, Eigen::Vector3d comPos)
{
    Eigen::Matrix4d T01, T12, T23, T34, T45;
    Eigen::Matrix4d T05;

    W = legIndex*W;
    l0y = legIndex*l0y;
    d12 = legIndex*d12;
    d2w = legIndex*d2w;

    T01 << rootOrient, comPos, 0, 0, 0, 1;
    T12 << mathOp.RotatePitch(jointAngles(0)), Eigen::Vector3d {L+l0x, W+l0y, H}, 0, 0, 0, 1;
    T23 << mathOp.RotatePitch(jointAngles(1)), Eigen::Vector3d {0, d12, l1}, 0, 0, 0, 1;
    T34 << mathOp.RotatePitch(jointAngles(0)+jointAngles(1)).transpose(), Eigen::Vector3d {0, d2w, l2}, 0, 0, 0, 1;
    T45 << Eigen::Matrix3d::Identity(), Eigen::Vector3d {0, 0, rw}, 0, 0, 0, 1;

    T05 = T01*T12*T23*T34*T45;
    contactPos << T05(0,3), T05(1,3), T05(2,3);
}

void robotLeg::solveLegIK(Eigen::Matrix3d rootOrient, Eigen::Vector3d footPos, Eigen::Vector3d comPos)
{

    W = legIndex*W;
    l0y = legIndex*l0y;
    d12 = legIndex*d12;
    d2w = legIndex*d2w;

    Eigen::Vector3d Rch, Rhf;
    Rch << L, W, H;
    Rhf = rootOrient.transpose()*(footPos - comPos) - Rch;

    double Cosq2 = (pow(Rhf(0)-l0x,2) + pow(Rhf(2) - rw,2) - l1*l1 - l2*l2)/(2*l1*l2);
    double Sinq2 = -sqrt(1-pow(Cosq2,2));
    double q2 = atan2(Sinq2,Cosq2);

    double A = l1*sin(q2);
    double B = l1 + l2*cos(q2);

    double Cosq1 = (A*(Rhf(0)-l0x) + B*(Rhf(2) - rw))/(pow(A,2) + pow(B,2));
    double Sinq1 = sqrt(1-pow(Cosq1,2));
    double q1 = atan2(Sinq1,Cosq1);
        
    qJoint << q1, q2;
}

void robotLeg::solveWheelIK(double Vrobot, double Wrobot)
{
    double Dw = 2*(l0y + d12 + d2w);
    wheelAngVel = (Vrobot - legIndex*0.5*Dw*Wrobot)/rw;
}



void twoLeggedWheeledRobot::solveFullBodyFK(Eigen::Vector3d comPos, Eigen::Matrix3d rootOrient, Eigen::Vector2d leftJointAngles, Eigen::Vector2d rightJointAngles)
{
    leftLeg.solveLegFK(rootOrient, leftJointAngles, comPos);
    rightLeg.solveLegFK(rootOrient, rightJointAngles, comPos);

    leftLegContactPos = leftLeg.contactPos;
    rightLegContactPos = rightLeg.contactPos;
}

void twoLeggedWheeledRobot::solveFullBodyIK(Eigen::Vector3d comPos, Eigen::Matrix3d rootOrient, Eigen::Vector3d leftFootPos, Eigen::Vector3d rightFootPos)
{
    leftLeg.solveLegIK(rootOrient, leftFootPos, comPos);
    rightLeg.solveLegIK(rootOrient, rightFootPos, comPos);
    
    jointPos << leftLeg.qJoint, rightLeg.qJoint;
}

void twoLeggedWheeledRobot::solveFullBodyWheelFK(Eigen::Vector2d wheels_AngVel)
{
    double Dw = 2*(l0y + d12 + d2w);
    Eigen::Vector2d vMat, wMat;
    vMat << 1, 1;
    wMat << 1, -1;
    robot_Vel = 0.5*rw*vMat.transpose()*wheels_AngVel;
    robot_AngVel = (Dw/rw)*wMat.transpose()*wheels_AngVel;
}

void twoLeggedWheeledRobot::solveFullBodyWheelIK(double Vrobot, double Wrobot)
{
    leftLeg.solveWheelIK(Vrobot, Wrobot);
    rightLeg.solveWheelIK(Vrobot, Wrobot);
    wheelsAngVel << leftLeg.wheelAngVel, rightLeg.wheelAngVel;
}

void twoLeggedWheeledRobot::jointPosController(Eigen::Vector4d measruedJointPos, double dT)
{
    jointController.jointLevelPD(jointPos, measruedJointPos, dT);
    tauJointPD = jointController.Tau_pd;
}

void twoLeggedWheeledRobot::wheelVelController(Eigen::Vector2d des_dqWheel, Eigen::Vector2d meas_qWheel, Eigen::Vector2d meas_dqWheel, double dT)
{
    jointController.wheelsPI(des_dqWheel, meas_qWheel, meas_dqWheel, dT);
    tauWheels << jointController.tauWheel_L, jointController.tauWheel_R;
}