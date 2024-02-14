package frc.robot.Logic;


import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Logic.StateMachine.robotStates;

public class AutoController {

    Robot thisRobot;
    int autoStep;
    public autoRoutines autoRoutine = autoRoutines.DO_NOTHING;
    public double autoStartTime;


    public AutoController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void autoInit(){

        autoRoutine = autoRoutines.valueOf(thisRobot.dashboard.autoSelector.getSelected());

        autoStartTime = Timer.getFPGATimestamp();
        autoStep = 0;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                thisRobot.onRedAlliance = true;
            }
            if (ally.get() == Alliance.Blue) {
                thisRobot.onRedAlliance = false;
            }
        }
        
    }

    public void autoPeriodic(){   

        switch (autoRoutine) {
            case DO_NOTHING:
                switch (autoStep) {
                    case 0:
                        thisRobot.stateMachine.robotState = robotStates.DRIVING;
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.swerveDrive.lockPose();

                        autoStep = 5;
                        break;

                    case 5:
                        //do nothing
                        break;
                
                    default:
                        break;
                }
                break;

            case PRELOAD_FROM_SIDE_AND_DRIVE_AWAY:

                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceSideToOpenSpace", thisRobot.onRedAlliance);
                        autoStep = 3;

                        break;

                    case 3:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 5;
                        break;

                    case 5:
                        thisRobot.stateMachine.updateRobotState();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || autoElapsedTime() > 2){
                            autoStep = 15;
                            thisRobot.stateMachine.robotState = robotStates.DRIVING;
                            thisRobot.stateMachine.updateRobotState();
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                            thisRobot.drivebase.setOdomToPathInit();
                            thisRobot.drivebase.followPath();
                        }
                        break;
                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }

                        break;

                    case 20:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;

                    default:
                        break;
                }
                
                break;

            case PRELOAD_FROM_MIDDLE_SCORE_1:

             switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleToNote1", thisRobot.onRedAlliance);
                        autoStep = 5;

                        break;

                     case 5:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 10;

                        break;
                    
                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || autoElapsedTime() > 2){
                            thisRobot.stateMachine.robotState = robotStates.INTAKING;
                            thisRobot.stateMachine.updateRobotState();
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                            thisRobot.drivebase.followPath();
                            autoStep = 13;
                        }
                        break;

                    case 13:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 15;
                        }
                        break;

                    case 15:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        //maybe come back and add diff case for the lock pose if shooter keeps running
                        break;

                    default:
                        break;
             }

             break;

            case PRELOAD_FROM_MIDDLE_SCORE_2:

                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleToNote2", thisRobot.onRedAlliance);
                        autoStep = 3;

                        break;

                    case 3:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        autoStep = 5;
                        break;

                    case 5:
                        thisRobot.stateMachine.updateRobotState();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || autoElapsedTime() > 2){
                            autoStep = 15;
                            thisRobot.stateMachine.robotState = robotStates.INTAKING;
                            thisRobot.stateMachine.updateRobotState();
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                            thisRobot.drivebase.followPath();
                        }
                        break;
                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }

                        break;

                    case 20:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        autoStep = 25;
                        break;

                    case 25:
                        thisRobot.stateMachine.updateRobotState();
                        break;
                }
          
            case PRELOAD_FROM_MIDDLE_SCORE_3:
                switch(autoStep) {
                
            case 0:
                thisRobot.drivebase.initPath("MiddleToNote3", thisRobot.onRedAlliance);
                    autoStep = 5;

                    break;

            case 5:
                thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                thisRobot.stateMachine.updateRobotState();
                    autoStep = 10;

                    break;

            case 10:
                 thisRobot.stateMachine.updateRobotState();
                 if(thisRobot.stateMachine.robotState == robotStates.DRIVING || autoElapsedTime() > 2){
                        autoStep = 15;
                        thisRobot.stateMachine.robotState = robotStates.INTAKING;
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        thisRobot.drivebase.followPath();
            }
            case 15:
                  thisRobot.drivebase.followPath();
                     if(thisRobot.drivebase.isPathOver()){
                        autoStep = 20;
                        }
            case 20:
                  thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                     thisRobot.stateMachine.updateRobotState();
                     thisRobot.drivebase.swerveDrive.lockPose();

                default:
                break;
     
          
        }

        case PRELOAD_FROM_MIDDLE_SCORE_1_2_3:
            switch(autoStep) {
        case 0:
                thisRobot.drivebase.initPath("MiddleToNote1", thisRobot.onRedAlliance);
                    autoStep = 5;

                    break;

            case 5:
                thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                thisRobot.stateMachine.updateRobotState();
                    autoStep = 10;

                    break;
                
            case 10:
                thisRobot.stateMachine.updateRobotState();
                if(thisRobot.stateMachine.robotState == robotStates.DRIVING || autoElapsedTime() > 2){
                        autoStep = 15;
                        thisRobot.stateMachine.robotState = robotStates.INTAKING;
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        thisRobot.drivebase.followPath();
            }  

            case 15:
             thisRobot.drivebase.followPath();
                     if(thisRobot.drivebase.isPathOver()){
                        autoStep = 20;
                        }
            case 20:
                thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                thisRobot.stateMachine.updateRobotState();
                autoStep = 25;

            case 25:
                if(thisRobot.stateMachine.robotState == robotStates.DRIVING || autoElapsedTime() > 2){
                        autoStep = 30;
                        thisRobot.stateMachine.robotState = robotStates.INTAKING;
                        thisRobot.stateMachine.updateRobotState();
                }

            case 30:
                 thisRobot.drivebase.initPath("MiddleToNote2", thisRobot.onRedAlliance);
                    autoStep = 35;

            case 35:
                thisRobot.stateMachine.robotState = robotStates.INTAKING;
                thisRobot.stateMachine.updateRobotState();
                    autoStep = 40;

            case 40:
                 thisRobot.drivebase.followPath();
                     if(thisRobot.drivebase.isPathOver()){
                        autoStep = 45;
                     }
            }
        thisRobot.updateAllSubsystemMotorPower();
    
    }
}

    public double autoElapsedTime(){
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    public enum autoRoutines {
        DO_NOTHING,
        PRELOAD_FROM_SIDE_AND_DRIVE_AWAY,
        PRELOAD_FROM_MIDDLE_SCORE_2,
        PRELOAD_FROM_MIDDLE_SCORE_3,
        PRELOAD_FROM_MIDDLE_SCORE_1,
        PRELOAD_FROM_MIDDLE_SCORE_1_2_3,

    }
}
