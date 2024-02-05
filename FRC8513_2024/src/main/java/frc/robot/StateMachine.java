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
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(5)){
                    robotState = RobotState.SPEEDING_UP_SHOOTER_SPEAKER;
                }
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(1)){
                    robotState = RobotState.INTAKING;
                }
                if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(2)){
                    robotState = RobotState.SPEEDING_UP_SHOOTER_AMP;
                }
                 if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(4)){
                    robotState = RobotState.SPEEDING_UP_SHOOTER_SPEAKER;
                }
                 if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(3)){
                    robotState = RobotState.SPEEDING_UP_SHOOTER_TRAP;
                }
                 if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(6)){
                    robotState = RobotState.CLIMB;
                }
                break;

            case INTAKING:
                
                break;

            case OUTAKING:

                break;

            case SPEEDING_UP_SHOOTER_SPEAKER:
                    robotState = RobotState.SHOOTING;
                
                break;

            case SPEEDING_UP_SHOOTER_AMP:
                 robotState = RobotState.SHOOTING;

                break;

            case SPEEDING_UP_SHOOTER_TRAP:
                 robotState = RobotState.SHOOTING;
                 
                break;

            case CLIMB:

                break;
        
            default:
                break;
        }
    }

    enum RobotState {
        DRIVING,
        INTAKING,
        OUTAKING,
        SHOOTING,
        SPEEDING_UP_SHOOTER_AMP,
        SPEEDING_UP_SHOOTER_SPEAKER,
        SPEEDING_UP_SHOOTER_TRAP,
        CLIMB
      }
}
