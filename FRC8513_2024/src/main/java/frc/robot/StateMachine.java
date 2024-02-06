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
                 if(thisRobot.teleopController.driverXboxController.getRawButtonPressed(4)){
                    robotState = RobotState.SPEEDING_UP_SHOOTER_SPEAKER;
                }
                break;

            case INTAKING:
                
                break;

            case SPEEDING_UP_SHOOTER_SPEAKER:
                    robotState = RobotState.SHOOTING;
                
                break;

            case CLIMB_PREP:

                break;

            case CLIMBING:

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
