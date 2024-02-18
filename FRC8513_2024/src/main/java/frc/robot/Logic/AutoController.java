package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        //get dashboard auto selector value
        autoRoutine = autoRoutines.valueOf(thisRobot.dashboard.autoSelector.getSelected());
        SmartDashboard.putString("AutoRoutine", autoRoutine.toString());

        //reset auto vars
        autoStartTime = Timer.getFPGATimestamp();
        autoStep = 0;

        //check DS to get what alliance we are on for flipping paths 
        thisRobot.updateAlliance();
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

            case Source_PD:
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

            case Mid_P2_D:

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
            break;

            case Mid_P231:
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleToNote1", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 10;
                        
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
                        break;
                        
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
                        break;

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
                    break;
                }
            break;    
    }
    thisRobot.updateAllSubsystemMotorPower();
}

    public double autoElapsedTime(){
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    public enum autoRoutines {
        DO_NOTHING,
        Amp_P,
        Mid_P,
        Source_P,
        Source_PD,
        Amp_P3_D,
        Mid_P2_D,
        Source_P1_D,
        Amp_P32_D,
        Mid_P23_D,
        Amp_P321,
        Mid_P231,
        Source_P123,
        Amp_P34,
        Source_P18,
        Mid_P26,
    }
}
