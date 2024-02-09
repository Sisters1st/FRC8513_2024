package frc.robot.Logic;


import frc.robot.Robot;
import frc.robot.Logic.StateMachine.robotStates;

public class AutoController {

    Robot thisRobot;
    int autoStep;
    public autoRoutines autoRoutine = autoRoutines.DO_NOTHING;


    public AutoController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void autoInit(){
        
        
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

            case SHOOT_PRELOAD_FROM_SOURCE_SIDE_AND_DRIVE_AWAY:

                switch (autoStep) {
                    case 0:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        autoStep = 5;
                        break;

                    case 5:
                        
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING){
                            autoStep = 10;
                        }

                        break;

                    case 10:
                        thisRobot.drivebase.initPath("SourceSideToOpenSpace", thisRobot.onBlueAlliance);
                        autoStep = 15;
                        
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }

                    case 20:
                        thisRobot.drivebase.swerveDrive.lockPose();
                    
                        break;

                    default:
                        break;
                }
                
                break;
        
            default:
                break;
        }

        thisRobot.updateAllSubsystemMotorPower();
    }

    public enum autoRoutines {
        DO_NOTHING,
        SHOOT_PRELOAD_FROM_SOURCE_SIDE_AND_DRIVE_AWAY
    }
}
