#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "control.hpp"

class robotLeg {
    private:

    protected:
        // Hip AA postion wrt CoM
        double L = 0;
        double W = 0.14;
        double H = 0;
 
        // link lengths
        double l0x = 0.0000;
        double l0y = 0.0000;
        double l1 = -0.2500; 
        double l2 = -0.2500;
        double rw = -0.1500;
        // distance between links
        double d12 = 0.0200;
        double d2w = 0.0225;

        int legIndex;

    public:
        Eigen::Vector3d contactPos;
        Eigen::Vector2d qJoint;
        double wheelAngVel;


        robotLeg(char leg);
        void solveLegFK(Eigen::Matrix3d rootOrient, Eigen::Vector2d jointAngles, Eigen::Vector3d comPos);
        void solveLegIK(Eigen::Matrix3d rootOrient, Eigen::Vector3d footPos, Eigen::Vector3d comPos);
        void solveWheelIK(double Vrobot, double Wrobot);


};

class twoLeggedWheeledRobot{
    private:      
        robotLeg leftLeg;
        robotLeg rightLeg;
    protected:
        double l0y = 0.0000;
        double d12 = 0.0200;
        double d2w = 0.0225;    
        double rw = 0.1500;

    public:
        twoLeggedWheeledRobot(): leftLeg('l'), rightLeg('r') {}

        Eigen::Vector3d leftLegContactPos, rightLegContactPos;
        void solveFullBodyFK(Eigen::Vector3d comPos, Eigen::Matrix3d rootOrient, Eigen::Vector2d leftJointAngles, Eigen::Vector2d rightJointAngles);

        Eigen::Vector4d jointPos;
        void solveFullBodyIK(Eigen::Vector3d comPos, Eigen::Matrix3d rootOrient, Eigen::Vector3d leftFootPos, Eigen::Vector3d rightFootPos);

        double robot_Vel, robot_AngVel;
        void solveFullBodyWheelFK(Eigen::Vector2d wheels_AngVel);

        Eigen::Vector2d wheelsAngVel;
        void solveFullBodyWheelIK(double Vrobot, double Wrobot);

        Eigen::Vector4d tauJointPD;
        void jointPosController(Eigen::Vector4d measruedJointPos, double dT);

        Eigen::Vector2d tauWheels;
        void wheelVelController(Eigen::Vector2d des_dqWheel, Eigen::Vector2d meas_qWheel, Eigen::Vector2d meas_dqWheel, double dT);
};

#endif