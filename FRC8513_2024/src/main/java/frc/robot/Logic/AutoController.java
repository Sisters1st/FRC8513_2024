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
                }
                break;

            case Source_PD:
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceToNote8", thisRobot.onRedAlliance);
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
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
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
                        thisRobot.drivebase.aimAtGoal();
                        autoStep = 25;

                    case 25:
                        thisRobot.stateMachine.updateRobotState();
                        break;
                }
            break;

            case Mid_P123:
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleToNote1", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 10;
                        
                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
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
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 25;

                    case 25:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
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

                    case 45:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 50;

                    case 50:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 55;
                                thisRobot.stateMachine.robotState = robotStates.INTAKING;
                                thisRobot.stateMachine.updateRobotState();
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleToNote3", thisRobot.onRedAlliance);
                        autoStep = 60;

                    case 60:
                        thisRobot.stateMachine.robotState = robotStates.INTAKING;
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 65;
                    break;
                    case 65:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 70;
                        }
                    break;
                    case 70:
                        thisRobot.stateMachine.robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        thisRobot.stateMachine.updateRobotState();
                        autoStep = 75;

                    case 75:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 80;
                                thisRobot.stateMachine.robotState = robotStates.INTAKING;
                                thisRobot.stateMachine.updateRobotState();
                        }
                    break;

                }
            break;   
            default:
            break;
        }
        thisRobot.updateAllSubsystemMotorPower();
    }

    public double autoElapsedTime(){
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    public boolean inSimAndTimePassedInState(double t){
        return (Robot.isSimulation() && ((Timer.getFPGATimestamp() - thisRobot.stateMachine.lastStateChangeTime) > t));
    }

    //_X means not made yet
    public enum autoRoutines {
        DO_NOTHING,
        _XAmp_P,
        _XMid_P,
        _XSource_P,
        Source_PD,
        _XAmp_P3_D,
        Mid_P2_D,
        _XSource_P1_D,
        _XAmp_P32_D,
        _XMid_P23_D,
        _XAmp_P321,
        Mid_P123,
        _XSource_P123,
        _XAmp_P34,
        _XSource_P18,
        _XMid_P26,
    }
}
