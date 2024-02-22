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
        
        //force shooter on
        thisRobot.stateMachine.forceShooterOn = true;
    }

    public void autoPeriodic(){   

        switch (autoRoutine) {
            case DO_NOTHING:
                switch (autoStep) {
                    case 0:
                        thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        thisRobot.drivebase.swerveDrive.lockPose();

                        autoStep = 5;
                        break;

                    case 5:
                        //do nothing
                        break;
                }
                break;

            
            case Amp_P3:
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("AmpStartToNote3ToAmpShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;
                        
                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 15;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                                thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }
                        break;
                        
                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 30;
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    }
                    break;

            case Mid_P2:
                //simulation looks good
                //works but needs to get tuned
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;
                        
                    case 10:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 15;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                                thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }
                        break;
                        
                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 30;
                        }
                        break;
                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    }
                    break;


            case Source_P1:
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceStartToNote1ToSourceShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;
                        
                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 15;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                                thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }
                        break;
                        
                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 30;
                        }
                        break;
                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    }
                    break;



            case Amp_P321:
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("AmpStartToNote3ToAmpShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;
                        
                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 15;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                                thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }
                        break;
                        
                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 30;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("AmpSideShotToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 55;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote1AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 80;
                                thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;
                }
                break;  

            case Mid_P123:
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote1ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;
                        
                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 15;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                                thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }
                        break;
                        
                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 30;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("MiddleShotToNote2AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 55;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote3AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 80;
                                thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;
                }
                break;  

            case Source_P123:
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceStartToNote1ToSourceShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;
                        
                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 15;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                                thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }
                        break;
                        
                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 30;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("SourceShotToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 55;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote3AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 80;
                                thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;
                }
                break;  
            
            case Amp_P34:
                switch(autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("AmpStartToNote3ToAmpShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;
                        
                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 15;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                                thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 20;
                        }
                        break;
                        
                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 30;
                                thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("AmpSideShotToNote4BackToShot", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if(thisRobot.stateMachine.robotState == robotStates.DRIVING || inSimAndTimePassedInState(1)){
                                autoStep = 55;
                                thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        //do nothing
                }
                break;
                
            
            case _XTESTINGACCURACY:
                switch(autoStep) {
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
                        if(thisRobot.drivebase.isPathOver()){
                            autoStep = 30;
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    }
                    break;
            default:
                break;
        }

        thisRobot.stateMachine.updateRobotState();
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
        Amp_P3,
        Mid_P2,
        Source_P1,
        _XAmp_P32_D,
        _XMid_P23_D,
        Amp_P321,
        Mid_P123,
        Source_P123,
        Amp_P34,
        _XSource_P18,
        _XMid_P26,
        _XTESTINGACCURACY
    }
}
