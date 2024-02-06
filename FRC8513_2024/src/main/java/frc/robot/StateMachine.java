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
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.shootInSpeakerButton)){
                    robotState = RobotState.SPEEDING_UP_SHOOTER_SPEAKER;
                }
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.intakeButton)){
                    robotState = RobotState.INTAKING;
                }
                 if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.shootInAMPWarmUpButton)){
                    robotState = RobotState.SPEEDING_UP_SHOOTER_SPEAKER;
                }
                break;

            case INTAKING:
                // thisRobot.arm.setArmPosition(Settings.intakingArmPosition);
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

            case CLIMB_PREP:
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.climberButton)){
                    robotState = RobotState.CLIMBING;
                }
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.drivingStateReturnButton)){
                    robotState = RobotState.DRIVING;
                }

                break;

            case CLIMBING:

                robotState = RobotState.CLIMB_TRAP;

                break;

            case CLIMB_TRAP:

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
        AMP_PREP,
        SCORE_AMP,
        CLIMB_PREP,
        CLIMBING,
        CLIMB_TRAP
      }
}
