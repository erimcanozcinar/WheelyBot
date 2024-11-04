#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "control.hpp"

#define MASS 7.66

class robotLeg : virtual protected mathOperations{
    private:

    protected:
        // Hip AA postion wrt CoM
        double L = 0;
        double W = 0.14;
        double H = 0;
 
        // link lengths
        double l0x = 0.00000;
        double l0y = 0.00000;
        double l1 = -0.23728; 
        double l2 = -0.25400;
        double rw = -0.25400;
        // distance between links
        double d12 = 0.02000;
        double d2w = 0.02250;

        int legIndex;

    public:
        Eigen::Vector3d contactPos;
        Eigen::Vector2d qJoint;
        double wheelAngVel;


        robotLeg(char leg);
        void solveLegFK(Eigen::Matrix3d rootOrient, Eigen::Vector2d jointAngles, Eigen::Vector3d comPos);
        void solveLegIK(Eigen::Matrix3d rootOrient, Eigen::Vector3d footPos, Eigen::Vector3d comPos);
        void solveWheelIK(double Vrobot, double Wrobot, double Wy);

        Eigen::Matrix2d legJacobianMat;
        void legJacobian(Eigen::Vector2d jointPos);


};

class twoLeggedWheeledRobot : protected jointLevelControllers, protected modelBasedControllers{
    private:      
        robotLeg leftLeg;
        robotLeg rightLeg;

        Eigen::Vector3d pre_unwrapedIMU;
    protected:
        double l0y = 0.0000;
        double d12 = 0.0200;
        double d2w = 0.0225;    
        double rw = 0.25400;

    public:
        twoLeggedWheeledRobot(): leftLeg('l'), rightLeg('r') {}

        Eigen::Vector3d leftLegContactPos, rightLegContactPos;
        double robot_Vel, robot_AngVel;
        void solveFullBodyFK(Eigen::Vector3d comPos, Eigen::Matrix3d rootOrient, Eigen::Vector2d leftJointAngles, Eigen::Vector2d rightJointAngles, Eigen::Vector2d wheels_AngVel);

        Eigen::Vector4d jointPos;
        Eigen::Vector2d wheelJointPos;
        Eigen::Vector2d wheelsAngVel;
        void solveFullBodyIK(Eigen::Vector3d comPos, Eigen::Matrix3d rootOrient, double Vrobot, double Wrobot, double Wy, Eigen::Vector3d leftFootPos, Eigen::Vector3d rightFootPos);

        Eigen::Vector3d unwrapedIMU, d_unwrapedIMU;
        void unwrapIMU(Eigen::Vector3d eulerIMU, Eigen::Vector3d pre_eulerIMU, double dt);


        Eigen::Vector4d tauJointPD;
        void jointPosController(Eigen::Vector4d measruedJointPos, double dT);

        Eigen::Matrix<double, 4, 2> jacobianMat;
        void fullBodyJacobian(Eigen::Vector4d meas_jointPos);
                
        Eigen::VectorXd stateVec{6}, stateVec_des{6};
        Eigen::Vector2d lqrTauWheels;
        void stabilizingController();

        Eigen::Matrix2d wheelJacInv;
        Eigen::Vector2d wheelTorques, ref_wheelVel;
        Eigen::Matrix2d mobileMassMat;
        void wheelAngVelController(Eigen::Vector2d wheelTorq, Eigen::Vector2d wheelPos, Eigen::Vector2d wheelVel, double dt);
        
};

class stateEstimators : protected twoLeggedWheeledRobot{
    private:
        // twoLeggedWheeledRobot wheeledBiped;

        double prev_robotVelocity, prev_robotPosition, prev_robotVelocityFilt;
        double rw = 0.25400;
    public:
        stateEstimators();

        double robotPosition;
        double robotVelocity, robotVelocityFilt, robotAngularVelocity;
        void comStates(Eigen::Vector2d wheelsAngVel, Eigen::Vector2d qwheels, double dt);
};

#endif