package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.StateMachine.robotStates;

public class AutoController {

    Robot thisRobot;
    int autoStep;
    public autoRoutines autoRoutine = autoRoutines.DO_NOTHING;
    public double autoStartTime;
    String path1 = "";
    String path2 = "";
    String path3 = "";

    public AutoController(Robot thisRobot_) {
        thisRobot = thisRobot_;
    }

    public void autoInit() {
        // get dashboard auto selector value
        autoRoutine = autoRoutines.valueOf(thisRobot.dashboard.autoSelector.getSelected());
        SmartDashboard.putString("AutoRoutine", autoRoutine.toString());

        // reset auto vars
        autoStartTime = Timer.getFPGATimestamp();
        autoStep = 0;

        // check DS to get what alliance we are on for flipping paths
        thisRobot.updateAlliance();

        // force shooter on
        thisRobot.stateMachine.forceShooterOn = true;
        path1 = "";
        path2 = "";
        path3 = "";
    }

    public void autoPeriodic() {

        switch (autoRoutine) {
            case DO_NOTHING:
                switch (autoStep) {
                    case 0:
                        thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        thisRobot.drivebase.swerveDrive.lockPose();

                        autoStep = 5;
                        break;

                    case 5:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;
            
            case Shoot_P:
                switch (autoStep){
                    case 0:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.dontShoot = true;
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 5;
                        break;

                    case 5:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        thisRobot.dontShoot = false;
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING){
                            autoStep = 10;
                        }
                        break;

                    case 10:
                        thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        thisRobot.drivebase.swerveDrive.lockPose();
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                    }
                break;



            case Amp_P3:
                // need sim
                 path1 = "AmpStartToNote3ToAmpShot";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Mid_P2:
                // need sim
                path1 = "MiddleStartToNote2ToMidShot";
                autoRoutine = autoRoutines._XGenericAuto;
                break;


            case Source_P1:
                // need sim
                path1 = "SourceStartToNote1ToSourceShot";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Source_P12:
                // need sim
                path1 = "SourceStartToNote1ToSourceShot";
                path2 = "SourceShotToNote2ToMidShot";
                autoRoutine = autoRoutines._XGenericAuto;
                break;


            case Amp_P321:
                // need sim
                path1 = "AmpStartToNote3ToAmpShot";
                path2 = "AmpSideShotToNote2ToMidShot";
                path3 = "MiddleShotToNote1AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;
            
            case Mid_P123:
                path1 = "MiddleStartToNote1ToMidShot";
                path2 = "MiddleShotToNote2AndBack";
                path3 = "MiddleShotToNote3AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Mid_P231:
                // need sim
                path1 = "MiddleStartToNote2ToMidShot";
                path2 = "MiddleShotToNote3AndBack";
                path3 = "MiddleShotToNote1AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;
            
            case Mid_P213:
                // need sim
                path1 = "MiddleStartToNote2ToMidShot";
                path2 = "MiddleShotToNote1AndBack";
                path3 = "MiddleShotToNote3AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Mid_P321:
                // need sim
                path1 = "MiddleStartToNote3ToMidShot";
                path2 = "MiddleShotToNote2AndBack";
                path3 = "MiddleShotToNote1AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Source_P123:
                // needs sim
                path1 = "SourceStartToNote1ToSourceShot";
                path2 = "SourceShotToNote2ToMidShot";
                path3 = "MiddleShotToNote3AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Amp_P34:
                // need sim
                path1 = "AmpStartToNote3ToAmpShot";
                path2 = "AmpSideShotToNote4BackToShot";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Amp_P35:
                // needs sim
                path1 = "AmpStartToNote3ToAmpShot";
                path2 = "AmpSideShotToNote5BackToShot";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Mid_P23:
                // needs sim
                path1 = "MiddeStartToNote2ToMidShot";
                path2 = "MiddleShotToNote3AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Mid_P21:
            // need to smulate
                path1 = "MiddeStartToNote2ToMidShot";
                path2 = "MiddleShotToNote1AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Amp_P32:
                // good in simulation
                path1 = "AmpStartToNote3ToAmpShot";
                path2 = "AmpSideShotToNote2ToMidShot";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Source_P18:
                path1 = "SourceStartToNote1ToSourceShot";
                path2 = "SourceShotToNote8AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Source_P17:
                path1 = "SourceStartToNote1ToSourceShot";
                path2 = "SourceShotToNote7AndBack";
                autoRoutine = autoRoutines._XGenericAuto;
                break;

            case Mid_P26:
                path1 = "MiddleStartToNote2ToMidShot";
                path2 = "MiddleShotToNote6AndBack";
                autoRoutine = autoRoutines._XGenericAuto;

                break;

            case _XTESTINGACCURACY:
                switch (autoStep) {
                    case 0:
                        thisRobot.stateMachine.forceShooterOn = false;
                        Settings.usePhoton = false;
                        thisRobot.drivebase.initPath("TestingPath", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        autoStep = 15;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 30;
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;
            
            case _XGenericAuto:
                // update this first
                switch (autoStep) {
                    case 0:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.dontShoot = true;
                        autoStep = 5;
                        break;

                    case 5:
                        thisRobot.drivebase.initPath(path1, thisRobot.onRedAlliance);
                        autoStep = 10;
                        break;


                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        thisRobot.dontShoot = false;
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.dontShoot = true;
                        thisRobot.drivebase.followPath();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING && thisRobot.drivebase.getDistFromLastPose() < Settings.shootingDist){
                            thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        }
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                            thisRobot.dontShoot=false;
                        }
                        break;

                    case 20:
                        if(thisRobot.stateMachine.robotState != robotStates.SHOOTING){
                            thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        }
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        if(path2 == ""){
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING); 
                        } else {
                            thisRobot.drivebase.initPath(path2, thisRobot.onRedAlliance);
                            autoStep = 40;
                        }

                    case 40:
                        thisRobot.dontShoot = true;
                        thisRobot.drivebase.followPath();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING && thisRobot.drivebase.getDistFromLastPose() < Settings.shootingDist){
                            thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        }
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                            thisRobot.dontShoot = false;
                        }
                        break;

                    case 45:
                        if(thisRobot.stateMachine.robotState != robotStates.SHOOTING){
                            thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        }
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        if(path3 == ""){
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING); 
                        } else {
                            thisRobot.drivebase.initPath(path3, thisRobot.onRedAlliance);
                            autoStep = 65;
                        }

                    case 65:
                        thisRobot.dontShoot = true;
                        thisRobot.drivebase.followPath();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING && thisRobot.drivebase.getDistFromLastPose() < Settings.shootingDist){
                            thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        }
                        if (thisRobot.drivebase.isPathOver()) {
                            thisRobot.dontShoot = false;
                            autoStep = 70;
                        }
                        break;

                    case 70:
                        if(thisRobot.stateMachine.robotState != robotStates.SHOOTING){
                            thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        }
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 80:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;


           
            default:
                break;
        }

        thisRobot.stateMachine.updateRobotState();
        thisRobot.updateAllSubsystemMotorPower();
    }

    public double autoElapsedTime() {
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    //if in simulation, use t. if on real field, move on from shot if it takes more than 2s to get up to speed
    public boolean timePassedInState(double t) {
        if(Robot.isSimulation()){
            return ((Timer.getFPGATimestamp() - thisRobot.stateMachine.lastStateChangeTime) > t);
        } else {
            return (Timer.getFPGATimestamp() - thisRobot.stateMachine.lastStateChangeTime) > 1.5;
        }
    }

    // _X means not made yet

    public enum autoRoutines {
        DO_NOTHING,
        Shoot_P,
        Amp_P3,
        Amp_P32,
        Amp_P34,
        Amp_P35,
        Amp_P321,
        Mid_P2,
        Mid_P21,
        Mid_P23,
        Mid_P26,
        Mid_P123,
        Mid_P213,
        Mid_P231,
        Mid_P321,
        Source_P1,
        Source_P12,
        Source_P17,
        Source_P18,
        Source_P123,
        _XGenericAuto,
        _XTESTINGACCURACY

    }
}
