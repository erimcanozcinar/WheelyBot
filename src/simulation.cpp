#include "simulation.hpp"

simulation::simulation()
{
    /* #region: Create RaiSim World */
    world.setTimeStep(0.001);    
    ground = world.addGround(0, "steel");
    // world.setGravity({0, 0, 0});

    wheelyBot = world.addArticulatedSystem("/home/erim/RaiSim_Simulations/wheelyBot/rsc/wheelyBot.urdf");
    wheelyBot->getCollisionBody("Wheel_L/0").setMaterial("rubber");
    wheelyBot->getCollisionBody("Wheel_R/0").setMaterial("rubber");

    world.setMaterialPairProp("steel", "steel", 0.95, 0.95, 0.001, 0.95, 0.001);
    world.setMaterialPairProp("steel", "rubber", 0.95, 0.15, 0.001, 0.95, 0.001);
    /* #endregion */
            
    Pcom << 0, 0, 0.4;
    Pcom_offset << 0, 0, 0.05;
    Pf_L << 0.0, 0.1825, 0.0;
    Pf_R << 0.0, -0.1825, 0.0;

    ref_dqWheel.setZero();

    genCoordinates.resize(13); genVelocities.resize(12); genAcceleration.resize(12); genForce.resize(12);
    commandTau.resize(6);

    /* #region: Initialize Robot */
    wheelyBotRobot.solveFullBodyIK(Pcom, Eigen::Matrix3d::Identity(), Pf_L, Pf_R);
    genCoordinates << Pcom+Pcom_offset, 1, 0, 0, 0, wheelyBotRobot.jointPos(0), wheelyBotRobot.jointPos(1), 0, wheelyBotRobot.jointPos(2), wheelyBotRobot.jointPos(3), 0;
    wheelyBot->setGeneralizedCoordinate(genCoordinates);
    /* #endregion */
}

void simulation::simulate()
{
    /* #region: Launch raisim server for visualization.Can be visualized on raisimUnity */
    raisim::RaisimServer server(&world);
    server.focusOn(wheelyBot);
    server.launchServer(); 
    /* #endregion */

    /* #region: Log file*/
    FILE* fp0;
    fp0 = fopen("../Log/dataLog.txt", "w");
    /* #endregion */

    while(true){
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();


        /* #region: READ ACTUAL DATA */
        genCoordinates = wheelyBot->getGeneralizedCoordinate().e();
        genVelocities = wheelyBot->getGeneralizedVelocity().e();
        genAcceleration = wheelyBot->getGeneralizedAcceleration().e();

        imuRot = mathOp.quat2euler(genCoordinates(3), genCoordinates(4), genCoordinates(5), genCoordinates(6));
        imuRotMat = mathOp.quat2RotMat(genCoordinates(3), genCoordinates(4), genCoordinates(5), genCoordinates(6));
        qJoint_enc << genCoordinates(7), genCoordinates(8), genCoordinates(10), genCoordinates(11);
        dqJoint_enc << genVelocities(6), genVelocities(7), genVelocities(9), genVelocities(10);
        qWheel_enc << genCoordinates(9), genCoordinates(12);
        dqWheel_enc << genVelocities(8), genVelocities(11);
        wheelyBotRobot.solveFullBodyWheelFK(dqWheel_enc);
        /* #endregion */     

        /* #region: INVERSE KINEMATICS*/        
        wheelyBotRobot.solveFullBodyIK(Pcom, Eigen::Matrix3d::Identity(), Pf_L, Pf_R);
        /* #endregion */



        /* #region: CONTROLLERS */
        torsoVelController.linearVelController(0, wheelyBotRobot.robot_Vel, dt);
        torsoStabilizer.torsoStabilization(torsoVelController.ref_pitch, imuRot, dt);
        wheelyBotRobot.jointPosController(qJoint_enc, dt);

        ref_dqWheel << torsoStabilizer.des_dqWheels*0.5, torsoStabilizer.des_dqWheels*0.5;
        wheelyBotRobot.wheelVelController(ref_dqWheel, qWheel_enc, dqWheel_enc, dt);

        /* #endregion */  
        
        /* #region: Send Torque Command to the Robot */
        commandTau << wheelyBotRobot.tauJointPD(0), wheelyBotRobot.tauJointPD(1), wheelyBotRobot.tauWheels(0), 
                      wheelyBotRobot.tauJointPD(2), wheelyBotRobot.tauJointPD(3), wheelyBotRobot.tauWheels(1);
        genForce << 0, 0, 0, 0, 0, 0, commandTau;
        wheelyBot->setGeneralizedForce(genForce);
        /* #endregion */

        fprintf(fp0, "%f %f %f\n", t, ref_dqWheel(0), dqWheel_enc(0));

        server.integrateWorldThreadSafe(); 
    }
    server.killServer();
    std::cout << "End of simulation" << std::endl;
}