#include "robot.hpp"

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
    T12 << RotatePitch(jointAngles(0)), Eigen::Vector3d {L+l0x, W+l0y, H}, 0, 0, 0, 1;
    T23 << RotatePitch(jointAngles(1)), Eigen::Vector3d {0, d12, l1}, 0, 0, 0, 1;
    T34 << RotatePitch(jointAngles(0)+jointAngles(1)).transpose(), Eigen::Vector3d {0, d2w, l2}, 0, 0, 0, 1;
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

    double A = l2*sin(q2);
    double B = l1 + l2*cos(q2);

    double Cosq1 = (A*(Rhf(0)-l0x) + B*(Rhf(2) - rw))/(pow(A,2) + pow(B,2));
    double Sinq1 = sqrt(1-pow(Cosq1,2));
    double q1 = atan2(Sinq1,Cosq1);
        
    qJoint << q1, q2;
}

void robotLeg::solveWheelIK(double Vrobot, double Wrobot, double Wy)
{
    double Dw = 2*(W + l0y + d12 + d2w);
    wheelAngVel = (Vrobot - legIndex*0.5*Dw*Wrobot - rw*Wy)/rw;
}

void robotLeg::legJacobian(Eigen::Vector2d jointPos)
{
    legJacobianMat << l2*cos(jointPos(0) + jointPos(1)) + l1*cos(jointPos(0) + 2*jointPos(1)), l2*cos(jointPos(0) + jointPos(1)) + 2*l1*cos(jointPos(0) + 2*jointPos(1)), 2, 2;
}


twoLeggedWheeledRobot::twoLeggedWheeledRobot() : leftLeg('l'), rightLeg('r')
{
    prevRef_wheelVel.setZero();
    prevRef_wheelAcc.setZero();
}

void twoLeggedWheeledRobot::solveFullBodyFK(Eigen::Vector3d comPos, Eigen::Matrix3d rootOrient, Eigen::Vector2d leftJointAngles, Eigen::Vector2d rightJointAngles, Eigen::Vector2d wheels_AngVel)
{
    double Dw = 2*(W + l0y + d12 + d2w);
    Eigen::Vector2d vMat, wMat;

    leftLeg.solveLegFK(rootOrient, leftJointAngles, comPos);
    rightLeg.solveLegFK(rootOrient, rightJointAngles, comPos);

    leftLegContactPos = leftLeg.contactPos;
    rightLegContactPos = rightLeg.contactPos;
    
    vMat << 1, 1;
    wMat << 1, -1;
    robot_Vel = 0.5*rw*vMat.transpose()*wheels_AngVel;
    robot_AngVel = (rw/Dw)*wMat.transpose()*wheels_AngVel;
}

void twoLeggedWheeledRobot::solveFullBodyIK(Eigen::Vector3d comPos, Eigen::Matrix3d rootOrient, double Vrobot, double Wrobot, double Wy, Eigen::Vector3d leftFootPos, Eigen::Vector3d rightFootPos)
{
    leftLeg.solveLegIK(rootOrient, leftFootPos, comPos);
    rightLeg.solveLegIK(rootOrient, rightFootPos, comPos);
    
    jointPos << leftLeg.qJoint, rightLeg.qJoint;
    wheelJointPos << -(leftLeg.qJoint(0)+leftLeg.qJoint(1)), -(rightLeg.qJoint(0)+rightLeg.qJoint(1));

    leftLeg.solveWheelIK(Vrobot, Wrobot, Wy);
    rightLeg.solveWheelIK(Vrobot, Wrobot, Wy);
    wheelsAngVel << leftLeg.wheelAngVel, rightLeg.wheelAngVel;
}

void twoLeggedWheeledRobot::unwrapIMU(Eigen::Vector3d eulerIMU, Eigen::Vector3d pre_eulerIMU, double dt)
{
    double imu_unwraped_roll = funcUnwrap(eulerIMU(0), pre_eulerIMU(0));
    double imu_unwraped_pitch = eulerIMU(1);
    double imu_unwraped_yaw = funcUnwrap(eulerIMU(2), pre_eulerIMU(2));
    unwrapedIMU << imu_unwraped_roll, imu_unwraped_pitch, imu_unwraped_yaw;

    d_unwrapedIMU << Numdiff(unwrapedIMU(0), pre_unwrapedIMU(0), dt),
                     Numdiff(unwrapedIMU(1), pre_unwrapedIMU(1), dt),
                     Numdiff(unwrapedIMU(2), pre_unwrapedIMU(2), dt);
    pre_unwrapedIMU = unwrapedIMU;
}

void twoLeggedWheeledRobot::jointPosController(Eigen::Vector4d measruedJointPos, double dT)
{
    jointLevelPD(jointPos, measruedJointPos, dT);
    tauJointPD = Tau_pd;
}

void twoLeggedWheeledRobot::fullBodyJacobian(Eigen::Vector4d meas_jointPos)
{
    Eigen::Vector2d leftLegJointPos, rightLegJointPos;
    leftLegJointPos << meas_jointPos(0), meas_jointPos(1);
    rightLegJointPos << meas_jointPos(2), meas_jointPos(3);
    leftLeg.legJacobian(leftLegJointPos);
    rightLeg.legJacobian(rightLegJointPos);
    jacobianMat << leftLeg.legJacobianMat.transpose(), rightLeg.legJacobianMat.transpose();

}

void twoLeggedWheeledRobot::stabilizingController()
{
    lqrStates = stateVec;
    lqrStates_des = stateVec_des;
    LQR();
    lqrTauWheels = lqrTau;
}

void twoLeggedWheeledRobot::wheelAngVelController(Eigen::Vector2d ref_wheelTorq, Eigen::Vector2d wheelsPos, Eigen::Vector2d wheelsVel, double dt)
{
    double Dw = 2*(W + l0y + d12 + d2w);
    mobileMassMatInv << 1/MASS, 0, 0, 1/0.05615;
    wheelJacInv << 1/rw,  Dw/(2*rw), 1/rw, -Dw/(2*rw);
    ref_wheelAcc = wheelJacInv*mobileMassMatInv*wheelJacInv.transpose()*ref_wheelTorq;
    for(int i=0; i<2; i++)
    {
        ref_wheelVel(i) = numIntegral(ref_wheelAcc(i), prevRef_wheelAcc(i), prevRef_wheelVel(i), dt);
    }
    prevRef_wheelVel = ref_wheelVel;
    prevRef_wheelAcc = ref_wheelAcc;

    wheelsPI(ref_wheelVel, wheelsPos, wheelsVel, dt);
    wheelTorques << tauWheel_L, tauWheel_R;
}


stateEstimators::stateEstimators()
{
    robotPosition = 0.0;
    prev_robotPosition = 0.0;
    prev_robotVelocity = 0.0;
    prev_robotVelocityFilt = 0.0;
};

void stateEstimators::comStates(Eigen::Vector2d wheelsAngVel, Eigen::Vector2d qwheels, double dt)
{
    solveFullBodyFK(Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero(),wheelsAngVel);
    robotVelocity = robot_Vel;
    robotAngularVelocity = robot_AngVel;

    robotVelocityFilt = LowPassFilter(robotVelocity, prev_robotVelocityFilt, 2*M_PI*50, dt);
    prev_robotVelocityFilt = robotVelocityFilt;
    
    robotPosition = numIntegral(robotVelocity, prev_robotVelocity, prev_robotPosition, dt);
    prev_robotVelocity = robotVelocity;
    prev_robotPosition = robotPosition;
}