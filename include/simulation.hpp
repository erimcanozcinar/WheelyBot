#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "robot.hpp"

class simulation{
    protected:
        raisim::World world;
        raisim::Ground *ground;
        raisim::ArticulatedSystem *wheelyBot;

        double t, dt;

        twoLeggedWheeledRobot wheelyBotRobot;
        stabilizationController torsoStabilizer;
        torsoVelControllers torsoVelController;
        mathOperations mathOp;        

        Eigen::Vector3d Pcom, torsoRot, Pcom_offset;
        Eigen::Vector3d Pf_L, Pf_R;
        Eigen::VectorXd genCoordinates, genVelocities, genAcceleration, genForce;

        Eigen::Vector3d imuRot;
        Eigen::Matrix3d imuRotMat;

        Eigen::Vector4d qJoint_enc, dqJoint_enc;
        Eigen::Vector2d qWheel_enc, dqWheel_enc;

        Eigen::Vector2d ref_dqWheel;

        Eigen::VectorXd commandTau ;
    
    public:
        simulation();
        void simulate();
};

#endif