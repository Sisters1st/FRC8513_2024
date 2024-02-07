package frc.robot;

public class StateMachine {

    Robot thisRobot;
    RobotState robotState = RobotState.DRIVING;
    
    public StateMachine(Robot robotIn){
        thisRobot = robotIn;
    }

    public void updateRobotState(){
        switch (robotState) {
            case DRIVING:

                thisRobot.arm.setArmPosition(Settings.intakingArmPos);
                thisRobot.wrist.setWristPositionToGround(Settings.intakingWristPos);
                thisRobot.intake.setIntakeVoltage(0);
                thisRobot.shooter.setShooterSpeeds(0, 0);
                thisRobot.shooter.feederMotor.setVoltage(0);
                
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.shootInSpeakerButton)){
                    robotState = RobotState.SPEEDING_UP_SHOOTER_SPEAKER;
                }
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.intakeButton)){
                    robotState = RobotState.INTAKING;
                }
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.climberPrepButton)){
                    robotState = RobotState.CLIMBING;
                }
                break;

            case INTAKING:
                thisRobot.arm.setArmPosition(Settings.intakingArmPos);
                thisRobot.wrist.setWristPositionToGround(Settings.intakingWristPos);
                thisRobot.intake.setIntakeVoltage(Settings.intakingVoltage);
                thisRobot.shooter.setShooterSpeeds(0, 0);
                thisRobot.shooter.feederMotor.setVoltage(Settings.feederIntakeVoltage);

                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.drivingStateReturnButton)){
                    robotState = RobotState.DRIVING;
                }

                break;

            case SPEEDING_UP_SHOOTER_SPEAKER:
                if((thisRobot.shooter.rightShooterInThreshold() && thisRobot.shooter.leftShooterInThreshold()) == true ){
                    robotState = RobotState.SHOOTING;
                }
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.drivingStateReturnButton)){
                    robotState = RobotState.DRIVING;
                }
                
                break;

            case CLIMBING:

                thisRobot.arm.setArmPosition(Settings.trapArmPos);
                thisRobot.wrist.setWristPositionToGround(Settings.trapWristPos);
                thisRobot.intake.setIntakeVoltage(0);
                thisRobot.shooter.setShooterSpeeds(0, 0);

                if(thisRobot.teleopController.operatingArmXboxController.getRawButtonPressed(Settings.drivingStateReturnButton)){
                    robotState = RobotState.DRIVING;
                }

                if(thisRobot.teleopController.driverXboxController.getRawButton(Settings.climbButton)){
                    thisRobot.climber.climberMotor1.setVoltage(Settings.climberVoltage);
                    thisRobot.climber.climberMotor2.setVoltage(Settings.climberVoltage);
                } else {
                    thisRobot.climber.climberMotor1.setVoltage(0);
                    thisRobot.climber.climberMotor2.setVoltage(0);
                }

                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.runFeederButton)){
                    thisRobot.shooter.feederMotor.setVoltage(Settings.feederScoreTrapVoltage);
                } else {
                    thisRobot.shooter.feederMotor.setVoltage(0);
                }

                break;

            case SCORE_AMP:

                thisRobot.arm.setArmPosition(Settings.ampArmPos);
                thisRobot.wrist.setWristPositionToGround(Settings.ampWristPos);
                thisRobot.intake.setIntakeVoltage(0);
                thisRobot.shooter.setShooterSpeeds(0, 0);

                if(thisRobot.teleopController.operatingArmXboxController.getRawButtonPressed(Settings.drivingStateReturnButton)){
                    robotState = RobotState.DRIVING;
                }

                if(thisRobot.teleopController.operatingArmXboxController.getRawButton(Settings.runFeederButton)){
                    thisRobot.shooter.feederMotor.setVoltage(Settings.feederScoreTrapVoltage);
                } else {
                    thisRobot.shooter.feederMotor.setVoltage(0);
                }

                break;
            default:
                break;
        }
    }

    enum RobotState {
        DRIVING,
        INTAKING,
        SHOOTING,
        SPEEDING_UP_SHOOTER_SPEAKER,
        SCORE_AMP,
        CLIMBING
      }
}
