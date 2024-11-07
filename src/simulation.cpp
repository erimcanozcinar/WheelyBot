#include "simulation.hpp"

simulation::simulation()
{
    /* #region: Create RaiSim World */
    world.setTimeStep(0.001);    
    ground = world.addGround(0, "steel");
    // world.setGravity({0,0,0});

    wheelyBot = world.addArticulatedSystem("/home/erim/RaiSim_Simulations/wheelyBot/rsc/wheelyBot.urdf");
    wheelyBot->getCollisionBody("Wheel_L/0").setMaterial("rubber");
    wheelyBot->getCollisionBody("Wheel_R/0").setMaterial("rubber");

    world.setMaterialPairProp("steel", "steel", 0.95, 0.95, 0.001, 0.95, 0.001);
    world.setMaterialPairProp("steel", "rubber", 0.95, 0.15, 0.001, 0.95, 0.001);
    /* #endregion */

    /* #region: Sensor update*/
    lordImu = wheelyBot->getSensor<raisim::InertialMeasurementUnit>("torso:imu");
    lordImu->setMeasurementSource(raisim::Sensor::MeasurementSource::RAISIM);
    /* #endregion*/
            
    Pcom << 0, 0, 0.65;
    Pcom_offset << 0, 0, 0.000;
    Pf_L << 0.0, 0.2175, 0.0;
    Pf_R << 0.0, -0.2175, 0.0;

    tP = 0; pre_tP = 0; pre_tVel = 0;

    ref_dqWheel.setZero();

    genCoordinates.resize(13); genVelocities.resize(12); genAcceleration.resize(12); genForce.resize(12);
    commandTau.resize(6);
    imuRot.setZero(); dimuRot.setZero(); pre_imuRot.setZero(), pre_unwrapedIMU.setZero();
    wheelyBotRobot.stateVec.setZero(); wheelyBotRobot.stateVec_des.setZero();
    wheelyBotRobot.lqrTauWheels.setZero();

    /* #region: Initialize Robot */
    wheelyBotRobot.solveFullBodyIK(Pcom, Eigen::Matrix3d::Identity(), 0, 0, 0, Pf_L, Pf_R);
    genCoordinates << Pcom+Pcom_offset, 1, 0, 0, 0, 
                      wheelyBotRobot.jointPos(0), wheelyBotRobot.jointPos(1), wheelyBotRobot.wheelJointPos(0), 
                      wheelyBotRobot.jointPos(2), wheelyBotRobot.jointPos(3), wheelyBotRobot.wheelJointPos(1);
    wheelyBot->setGeneralizedCoordinate(genCoordinates);
    /* #endregion */
}

void simulation::simulate()
{
    /* #region: Launch raisim server for visualization.Can be visualized on raisimUnity */
    raisim::RaisimServer server(&world);
    server.focusOn(wheelyBot);
    server.launchServer();
    raisim::MSLEEP(2000);
    /* #endregion */

    /* #region: Log file*/
    FILE* fp0; FILE* fp1;
    fp0 = fopen("../Log/dataLog.txt", "w");
    fp1 = fopen("../Log/dataLog2.txt", "w");
    /* #endregion */

    while(true){
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();

        /* #region: READ ACTUAL DATA */
        genCoordinates = wheelyBot->getGeneralizedCoordinate().e();
        genVelocities = wheelyBot->getGeneralizedVelocity().e();
        genAcceleration = wheelyBot->getGeneralizedAcceleration().e();
        
        imuRot = quat2euler(lordImu->getOrientation()[0], lordImu->getOrientation()[1], lordImu->getOrientation()[2], lordImu->getOrientation()[3]);
        wheelyBotRobot.unwrapIMU(imuRot, pre_imuRot, dt);
        pre_imuRot = imuRot;
        imuRotMat = quat2RotMat(genCoordinates(3), genCoordinates(4), genCoordinates(5), genCoordinates(6));
        
        qJoint_enc << genCoordinates(7), genCoordinates(8), genCoordinates(10), genCoordinates(11);
        dqJoint_enc << genVelocities(6), genVelocities(7), genVelocities(9), genVelocities(10);
        qWheel_enc << genCoordinates(9), genCoordinates(12);
        dqWheel_enc << genVelocities(8), genVelocities(11);
        /* #endregion */     

        /* #region: INVERSE KINEMATICS*/
        wheelyBotRobot.solveFullBodyIK(Pcom, Eigen::Matrix3d::Identity(), 0, 0, 0, Pf_L, Pf_R);
        /* #endregion */

        /* #region: CONTROLLERS */
        torsoEstimator.comStates(dqWheel_enc, qWheel_enc, dt);
        comPos << genCoordinates(0), genCoordinates(1), genCoordinates(2);
        comVel << genVelocities(0), genVelocities(1), genVelocities(2);
        comVel = RotateYaw(wheelyBotRobot.unwrapedIMU(2)).transpose()*comVel;
        comPos = RotateYaw(wheelyBotRobot.unwrapedIMU(2)).transpose()*comPos;
        trajGen.FuncPoly3rd(t, 15.0, 25.0, 0.0, 0.0, 3, 0.0);
        tP = numIntegral(trajGen.trajOut(0), pre_tVel, pre_tP, dt);
        pre_tP = tP; pre_tVel = trajGen.trajOut(0);

        if(t>25){
            trajTurn = (20*M_PI/180)*sin(2*M_PI*0.2*t);
            dtrajTurn = 2*M_PI*0.2*(20*M_PI/180)*cos(2*M_PI*0.2*t);
        }else{
            trajTurn = 0.0;
            dtrajTurn = 0.0;
        }

        wheelyBotRobot.jointPosController(qJoint_enc, dt);
        wheelyBotRobot.stateVec << wheelyBotRobot.unwrapedIMU(1), torsoEstimator.robotPosition, wheelyBotRobot.unwrapedIMU(2), wheelyBotRobot.d_unwrapedIMU(1), comVel(0), wheelyBotRobot.d_unwrapedIMU(2);
        wheelyBotRobot.stateVec_des << 0, tP, trajTurn, 0, trajGen.trajOut(0), dtrajTurn;
        wheelyBotRobot.stabilizingController();
        wheelyBotRobot.wheelAngVelController(wheelyBotRobot.lqrTauWheels, qWheel_enc, dqWheel_enc, dt);        
        /* #endregion */  
        
        /* #region: Send Torque Command to the Robot */
        commandTau << wheelyBotRobot.tauJointPD(0), wheelyBotRobot.tauJointPD(1), wheelyBotRobot.lqrTauWheels(0), 
                      wheelyBotRobot.tauJointPD(2), wheelyBotRobot.tauJointPD(3), wheelyBotRobot.lqrTauWheels(1);
        genForce << 0, 0, 0, 0, 0, 0, commandTau;
        wheelyBot->setGeneralizedForce(genForce);
        /* #endregion */
        // RSINFO(wheelyBot->getCOM())
        
        // fprintf(fp0, "%f %f %f %f %f %f %f\n", t, commandTau(0), commandTau(1), commandTau(2), commandTau(3), commandTau(4), commandTau(5));
        // fprintf(fp1, "%f %f %f %f %f %f %f\n", t, dqJoint_enc(0), dqJoint_enc(1), dqJoint_enc(2), dqJoint_enc(3), dqWheel_enc(0), dqWheel_enc(1));
        fprintf(fp0, "%f %f %f %f %f %f %f\n", t, wheelyBotRobot.ref_wheelVel(0), wheelyBotRobot.ref_wheelVel(1), dqWheel_enc(0), dqWheel_enc(1), commandTau(4), commandTau(5));

        server.integrateWorldThreadSafe(); 
    }
    server.killServer();
    std::cout << "End of simulation" << std::endl;
}