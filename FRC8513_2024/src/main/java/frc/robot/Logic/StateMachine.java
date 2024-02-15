package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;

public class StateMachine {

    Robot thisRobot;
    public robotStates robotState = robotStates.DRIVING;    
    double lastStateChangeTime = 0;
    double feederV = 0;

    public StateMachine(Robot robotIn){
        thisRobot = robotIn;
    }

    public void updateRobotState(){

        switch (robotState) {
            case DRIVING:

                thisRobot.arm.setArmPosition(Settings.intakingArmPos);
                thisRobot.wrist.setWristPos(Settings.intakingWristPos);
                thisRobot.intake.setIntakeVoltage(0);
                thisRobot.shooter.setShooterSpeeds(0, 0,0);
                
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.shootInSpeakerButton)){
                    robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.intakeButton)){
                    robotState = robotStates.INTAKING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.climberPrepButton)){
                    robotState = robotStates.CLIMBING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.ampPrepButton)){
                    robotState = robotStates.SCORE_AMP;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                break;

            case INTAKING:
                thisRobot.arm.setArmPosition(Settings.intakingArmPos);
                thisRobot.wrist.setWristPos(Settings.intakingWristPos);
                thisRobot.intake.setIntakeVoltage(Settings.intakingVoltage);
                thisRobot.shooter.setShooterSpeeds(0,0, Settings.feederIntakeVoltage);

                //evetually add sensor stop to go abck to driving
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.drivingStateReturnButton)){
                    robotState = robotStates.DRIVING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                
                break;

            case SPEEDING_UP_SHOOTER_SPEAKER:
                    
                thisRobot.arm.setArmPosition(Settings.shootingArmPos);
                thisRobot.wrist.setWristPos(Settings.shootingWristPos);
                thisRobot.intake.setIntakeVoltage(0);

                if(thisRobot.shooter.rightShooterInThreshold() && thisRobot.shooter.leftShooterInThreshold()  &&
                    thisRobot.arm.armWithinThold() && thisRobot.wrist.wristWithinThold()){
                    thisRobot.shooter.setShooterSpeeds(Settings.basicShooterSpeed, Settings.basicShooterSpeed, Settings.feederIntakeVoltage);
                          //after some time, go back to driving
                          
                } else {
                    thisRobot.shooter.setShooterSpeeds(Settings.basicShooterSpeed, Settings.basicShooterSpeed, 0);
                }

              
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.drivingStateReturnButton)){
                    robotState = robotStates.DRIVING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                
                 if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.intakeButton)){
                    robotState = robotStates.INTAKING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                
                break;


            case CLIMBING:

                thisRobot.arm.setArmPosition(Settings.trapArmPos);
                thisRobot.wrist.setWristPos(Settings.trapWristPos);
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.drivingStateReturnButton)){
                    robotState = robotStates.DRIVING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }

                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.climbButton)){
                    thisRobot.climber.climberMotor1.setVoltage(Settings.climberVoltage);
                    thisRobot.climber.climberMotor2.setVoltage(Settings.climberVoltage);
                } else {
                    thisRobot.climber.climberMotor1.setVoltage(0);
                    thisRobot.climber.climberMotor2.setVoltage(0);
                }

                feederV = 0;
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.runFeederOutButton)){
                    feederV = Settings.feederScoreTrapVoltage;
                } else {
                    feederV = 0;
                }

                thisRobot.shooter.setShooterSpeeds(0, 0, feederV);

                break;

            case SCORE_AMP:
                
                thisRobot.arm.setArmPosition(Settings.ampArmPos);
                thisRobot.wrist.setWristPos(Settings.ampWristPos);

                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.drivingStateReturnButton)){
                    robotState = robotStates.DRIVING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }
                
                feederV = 0;
                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.runFeederOutButton)){
                    feederV = Settings.feederScoreTrapVoltage;
                } else {
                    feederV = 0;
                }

                thisRobot.shooter.setShooterSpeeds(0, 0, feederV);

                 if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.intakeButton)){
                    robotState = robotStates.INTAKING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                }

                break;
            default:
                break;
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
