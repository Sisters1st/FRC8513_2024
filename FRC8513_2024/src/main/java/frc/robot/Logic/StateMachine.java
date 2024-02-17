package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;

public class StateMachine {

    Robot thisRobot;
    public robotStates robotState = robotStates.DRIVING;    
    double lastStateChangeTime = 0;
    double shotStartedTime = -1;

    //each of the subsystem vars to keep track of
    double feederV = 0;
    double armPos = 0;
    double wristPos = 0;
    double intakeVoltage = 0;
    double lss = 0;
    double rss = 0;

    public StateMachine(Robot robotIn){
        thisRobot = robotIn;
    }

    public void updateRobotState(){

        switch (robotState) {
            case DRIVING:
                armPos = Settings.intakingArmPos;
                wristPos = Settings.intakingWristPos;
                lss = rss = feederV = intakeVoltage = 0;
                freeFeederControl();
                checkAllButtonsForStateChanges();
                freeIntakeControl();
                
                break;

            case INTAKING:
                armPos = Settings.intakingArmPos;
                wristPos = Settings.intakingWristPos;
                intakeVoltage = Settings.intakingVoltage;
                feederV = Settings.feederIntakeVoltage;
                lss = rss = 0;

                //if sensor gets tripped, go back to driving
                if(thisRobot.shooter.intakeSensorSeesNote()){
                    robotState = robotStates.DRIVING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                checkAllButtonsForStateChanges();
                
                break;

            case SPEEDING_UP_SHOOTER_SPEAKER:
                    
                armPos = Settings.shootingArmPos;
                wristPos = thisRobot.wrist.getWristPos() + 0.1 * thisRobot.teleopController.manualControlJoystick.getRawAxis(Settings.manualControlWristAxis);

                feederV = intakeVoltage = 0;
                
                checkAllButtonsForStateChanges();

                if((thisRobot.shooter.rightShooterInThreshold() && thisRobot.shooter.leftShooterInThreshold()  &&
                    thisRobot.arm.armWithinThold() && thisRobot.wrist.wristWithinThold())){
                   
                    lss = rss = Settings.basicShooterSpeed;
                    feederV = Settings.feederIntakeVoltage;

                    if(shotStartedTime == -1){
                        shotStartedTime = Timer.getFPGATimestamp();
                    }
                    if(Timer.getFPGATimestamp() - shotStartedTime > Settings.shotTime){
                        robotState = robotStates.DRIVING;
                    }
                          
                } else {
                    shotStartedTime = -1;
                    lss = rss = Settings.basicShooterSpeed;
                    feederV = 0;
                }

                freeIntakeControl();
                freeFeederControl();
                break;

            case CLIMBING:

                armPos = Settings.trapArmPos;
                wristPos = Settings.trapWristPos;
                lss = rss = feederV = intakeVoltage = 0;

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
                lss = rss = feederV = intakeVoltage = 0;
                freeFeederControl();
                checkAllButtonsForStateChanges();
                freeIntakeControl();
                break;

            default:
                break;
        }

        thisRobot.arm.setArmPosition(armPos);
        thisRobot.wrist.setWristPos(wristPos);
        thisRobot.intake.setIntakeVoltage(intakeVoltage);
        thisRobot.shooter.setShooterSpeeds(lss, rss, feederV);

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

    public enum robotStates {
        DRIVING,
        INTAKING,
        SHOOTING,
        SPEEDING_UP_SHOOTER_SPEAKER,
        SCORE_AMP,
        CLIMBING
      }
}
