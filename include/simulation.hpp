#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "robot.hpp"
#include "trajectory.hpp"

class simulation : virtual protected mathOperations{
    protected:
        raisim::World world;
        raisim::Ground *ground;
        raisim::ArticulatedSystem *wheelyBot;

        raisim::InertialMeasurementUnit *lordImu;

        double t, dt;

        twoLeggedWheeledRobot wheelyBotRobot;
        stateEstimators torsoEstimator;
        // mathOperations mathOp;
        trajectoryGeneration trajGen, turnTraj;     

        Eigen::Vector3d Pcom, torsoRot, Pcom_offset, comPos, comVel;
        Eigen::Vector3d Pf_L, Pf_R;
        Eigen::VectorXd genCoordinates, genVelocities, genAcceleration, genForce;

        Eigen::Vector3d imuRot, pre_imuRot, dimuRot, pre_unwrapedIMU;
        Eigen::Matrix3d imuRotMat;        

        Eigen::Vector4d qJoint_enc, dqJoint_enc;
        Eigen::Vector2d qWheel_enc, dqWheel_enc;

        Eigen::Vector2d ref_dqWheel;

        Eigen::VectorXd commandTau ;

        double tP, pre_tP, pre_tVel;
        double trajTurn, dtrajTurn;
    
    public:
        simulation();
        void simulate();
};

#endif