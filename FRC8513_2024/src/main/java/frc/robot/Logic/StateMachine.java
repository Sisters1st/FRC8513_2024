package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;

public class StateMachine {

    Robot thisRobot;
    public robotStates robotState = robotStates.DRIVING;    
    double lastStateChangeTime = 0;
    double shotStartedTime = -1;
    boolean comittedToShot = false;
    double noteHitSensorTime = 0;

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
            case DRIVING:
                armPos = Settings.intakingArmPos;
                wristPos = Settings.intakingWristPos;
                ss = feederV = intakeVoltage = 0;
                freeFeederControl();
                checkAllButtonsForStateChanges();
                freeIntakeControl();
                
                break;

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
                checkAllButtonsForStateChanges();
                break;

            case SPEEDING_UP_SHOOTER_SPEAKER:
                    
                armPos = Settings.shootingArmPos;
                wristPos = getWristAngFromDist(thisRobot.shooter.getDistFromGoal());

                feederV = intakeVoltage = 0;
                
                checkAllButtonsForStateChanges();
                shotStartedTime = -1;
                ss = Settings.basicShooterSpeed;
                feederV = 0;
                comittedToShot = false;

                robotState = robotStates.SHOOTING;

                freeIntakeControl();
                freeFeederControl();
                break;

            case SHOOTING:
                armPos = Settings.shootingArmPos;
                wristPos = getWristAngFromDist(thisRobot.shooter.getDistFromGoal());
                feederV = intakeVoltage = 0;
                
                checkAllButtonsForStateChanges();
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

                freeIntakeControl();
                freeFeederControl();
                break;
            case CLIMBING:

                armPos = Settings.trapArmPos;
                wristPos = Settings.trapWristPos;
                ss = feederV = intakeVoltage = 0;

                checkAllButtonsForStateChanges();
                freeFeederControl();
                freeIntakeControl();

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
                freeFeederControl();
                checkAllButtonsForStateChanges();
                freeIntakeControl();
                break;

            default:
                break;
        }

        thisRobot.arm.setArmPosition(armPos);
        thisRobot.wrist.setWristPos(wristPos);
        double timeSinceLastSensorHit = Timer.getFPGATimestamp() - noteHitSensorTime;
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

    public void freeFeederControl(){
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.runFeederInButton)){
            feederV = Settings.feederIntakeVoltage;
        }
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.runFeederOutButton)){
            feederV = Settings.feederScoreAmpVoltage;
        }
    }

    public void freeIntakeControl(){
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.intakeOutButton)){
            intakeVoltage = -Settings.intakingVoltage;
        }
    }

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

    public double getWristAngFromDist(double dist){
        double a3 = 2.6;
        double a2 = -9.32;
        double a1 = 1.44; 
        double a0 = 1.62;

        double wristVal = a3 * Math.pow(dist, 3) + a2 * Math.pow(dist, 2) + a1 * dist + a0;
        return wristVal;
    }

    public boolean robotInAllTHolds(){
        return thisRobot.shooter.rightShooterInThreshold() && thisRobot.shooter.leftShooterInThreshold()  &&
                    thisRobot.arm.armWithinThold() && thisRobot.wrist.wristWithinThold() && thisRobot.drivebase.inHeadingThold();
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
