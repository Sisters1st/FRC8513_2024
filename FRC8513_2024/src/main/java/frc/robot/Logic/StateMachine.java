package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;

public class StateMachine {

    Robot thisRobot;
    public robotStates robotState = robotStates.DRIVING;    
    double lastStateChangeTime = 0; //track last time state changed
    double shotStartedTime = -1; //track when a shot was initiated
    boolean comittedToShot = false; //once we start a shot, we need to finish it
    double noteHitSensorTime = 0; //time for shimmy

    //each of the subsystem vars to keep track of
    double feederV = 0;
    double armPos = 0;
    double wristPos = 0;
    double intakeVoltage = 0;
    double ss = 0;

    public StateMachine(Robot robotIn){
        thisRobot = robotIn;
    }

    public void updateRobotState(){

        switch (robotState) {
            //driving around, intake off
            case DRIVING:
                armPos = Settings.intakingArmPos;
                wristPos = Settings.intakingWristPos;
                ss = feederV = intakeVoltage = 0;
                
                break;

            //run the intake, feeder in 
            case INTAKING:
                armPos = Settings.intakingArmPos;
                wristPos = Settings.intakingWristPos;
                intakeVoltage = Settings.intakingVoltage;
                feederV = Settings.feederIntakeVoltage;
                ss = 0;

                //if sensor gets tripped, go back to driving
                if(thisRobot.shooter.intakeSensorSeesNote()){
                    robotState = robotStates.DRIVING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                    noteHitSensorTime = Timer.getFPGATimestamp();
                }
                break;

            //runs once when we initilize a shot
            case SPEEDING_UP_SHOOTER_SPEAKER:
                
                //set arm and wrist based off shot distance
                armPos = Settings.shootingArmPos;
                wristPos = getWristAngFromDist(thisRobot.shooter.getDistFromGoal());

                feederV = intakeVoltage = 0;
                shotStartedTime = -1;
                ss = Settings.basicShooterSpeed;
                feederV = 0;
                comittedToShot = false;

                //move on from the shot
                robotState = robotStates.SHOOTING;

                break;

            case SHOOTING:
                armPos = Settings.shootingArmPos;
                wristPos = getWristAngFromDist(thisRobot.shooter.getDistFromGoal());
                feederV = intakeVoltage = 0;
                
                feederV = 0;

                if(robotInAllTHolds() || comittedToShot){
                   
                    ss = Settings.basicShooterSpeed;
                    feederV = Settings.feederIntakeVoltage;

                    if(shotStartedTime == -1){
                        shotStartedTime = Timer.getFPGATimestamp();
                        comittedToShot = true;
                    }
                    
                    if(Timer.getFPGATimestamp() - shotStartedTime > Settings.shotTime){
                        robotState = robotStates.DRIVING;
                    }
                          
                } else {
                    ss = Settings.basicShooterSpeed;
                    feederV = 0;
                }

                break;
            case CLIMBING:

                armPos = Settings.trapArmPos;
                wristPos = Settings.trapWristPos;
                ss = feederV = intakeVoltage = 0;

                if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbUpButton)){
                    thisRobot.climber.climberMotor1.setVoltage(Settings.climberVoltage);
                    thisRobot.climber.climberMotor2.setVoltage(Settings.climberVoltage);
                } else {
                    if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbDownButton))
                    {
                        thisRobot.climber.climberMotor1.setVoltage(-Settings.climberVoltage);
                        thisRobot.climber.climberMotor2.setVoltage(-Settings.climberVoltage);
                    } else {
                        thisRobot.climber.climberMotor1.setVoltage(0);
                        thisRobot.climber.climberMotor2.setVoltage(0);
                    }
                }
                break;

            case SCORE_AMP:
                
                armPos = Settings.ampArmPos;
                wristPos = Settings.ampWristPos;
                ss = feederV = intakeVoltage = 0;

                break;

            default:
                break;
        }
        checkAllButtonsForStateChanges();
        if(robotState != robotStates.INTAKING){
            freeFeederControl();
            freeIntakeControl();
        }

        thisRobot.arm.setArmPosition(armPos);
        thisRobot.wrist.setWristPos(wristPos);
        double timeSinceLastSensorHit = Timer.getFPGATimestamp() - noteHitSensorTime;
        //may have to add logic which brings note back to sensor after shimmy
        if(timeSinceLastSensorHit < Settings.noteShimmyTime){
            int shimmyCount = (int)(timeSinceLastSensorHit * 5);
            if(shimmyCount % 2 == 0){
                thisRobot.intake.setIntakeVoltage(Settings.feederIntakeVoltage);
            } else {
                thisRobot.intake.setIntakeVoltage(-Settings.feederIntakeVoltage);
            }
        } else {
            thisRobot.intake.setIntakeVoltage(intakeVoltage);
        }
        thisRobot.intake.setIntakeVoltage(intakeVoltage);
        thisRobot.shooter.setShooterSpeeds(ss, feederV);

    }

    //manual control of the feeder
    public void freeFeederControl(){
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.runFeederInButton)){
            feederV = Settings.feederIntakeVoltage;
        }
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.runFeederOutButton)){
            feederV = Settings.feederScoreAmpVoltage;
        }
    }

    //manual spit out of intake
    public void freeIntakeControl(){
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.intakeOutButton)){
            intakeVoltage = -Settings.intakingVoltage;
        }
    }

    //from any state, check if we need to go into another state
    public void checkAllButtonsForStateChanges(){
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.shootInSpeakerButton)){
            robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.intakeButton)){
            robotState = robotStates.INTAKING;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.climberPrepButton)){
            robotState = robotStates.CLIMBING;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.ampPrepButton)){
            robotState = robotStates.SCORE_AMP;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.drivingStateReturnButton)){
            robotState = robotStates.DRIVING;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
    }

    //generated from cubic line of best fit. will need to get retuned
    public double getWristAngFromDist(double dist){
        double a3 = 2.6;
        double a2 = -9.32;
        double a1 = 1.44; 
        double a0 = 1.62;

        double wristVal = a3 * Math.pow(dist, 3) + a2 * Math.pow(dist, 2) + a1 * dist + a0;
        return wristVal;
    }

    public boolean robotInAllTHolds(){
        return thisRobot.shooter.shootersWithinThold()
        && thisRobot.arm.armWithinThold() 
        && thisRobot.wrist.wristWithinThold() 
        && thisRobot.drivebase.inHeadingThold()
        && thisRobot.drivebase.inVThold()
        && thisRobot.shooter.shotWithinRange();
    }

    public enum robotStates {
        DRIVING,
        INTAKING,
        SHOOTING,
        SPEEDING_UP_SHOOTER_SPEAKER,
        SCORE_AMP,
        CLIMBING
      }
}
