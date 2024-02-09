package frc.robot.Logic;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.StateMachine.RobotState;

public class TeleopController {

    Robot thisRobot;
    Joystick driverXboxController = new Joystick(Settings.driverJoystickPort);
    Joystick operatingArmXboxController = new Joystick(Settings.opperatingArmJoystickPort);
    Joystick climberXboxController = new Joystick(Settings.climberJoystickPort);

    public TeleopController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void init(){
        thisRobot.stateMachine.robotState = RobotState.DRIVING;
        thisRobot.arm.setArmPosition(thisRobot.arm.getArmPosition());
        thisRobot.wrist.setWristPos(thisRobot.wrist.getWristPos());
    }

    public void periodic(){
        boolean manualTestingControl = false;
        if(manualTestingControl){
            manualControl();

        } else {
            thisRobot.stateMachine.updateRobotState();
            thisRobot.arm.updateArmMotorPower();
            thisRobot.wrist.updateWristMotorPower();
        }
        
        driveTele();
    }

    public void driveTele(){
        double xSpeedJoystick = -driverXboxController.getRawAxis(1); //forward back
        if(xSpeedJoystick < Settings.joyBand && xSpeedJoystick > -Settings.joyBand){
            xSpeedJoystick = 0;
        }
        double ySpeedJoystick = -driverXboxController.getRawAxis(0); //left right
         if(ySpeedJoystick < Settings.joyBand && ySpeedJoystick > -Settings.joyBand){
            ySpeedJoystick = 0;
        }
        double rSpeedJoystick = -driverXboxController.getRawAxis(4); //left right 2 at home, 4 on xbox
         if(rSpeedJoystick < Settings.joyBand && rSpeedJoystick > -Settings.joyBand){
            rSpeedJoystick = 0;
        }

        double xInput = Math.pow(xSpeedJoystick, 3); // Smooth controll out
        double yInput = Math.pow(ySpeedJoystick, 3); // Smooth controll out

        double xV = xInput * thisRobot.drivebase.swerveDrive.getMaximumVelocity();
        double yV = yInput * thisRobot.drivebase.swerveDrive.getMaximumVelocity();
        double rV = rSpeedJoystick * thisRobot.drivebase.swerveDrive.getMaximumAngularVelocity();

        thisRobot.drivebase.swerveDrive.drive(
            new Translation2d(xV, yV),
            rV,
            true,
            false
        );
    }

    public void manualControl(){

        if(operatingArmXboxController.getRawButton(5)){
            thisRobot.arm.armMotor1.getEncoder().setPosition(0);
            
            thisRobot.wrist.wristMotor1.getEncoder().setPosition(0);
        }

        double climberJoystick = -climberXboxController.getRawAxis(0); //Up Down????
        if(climberJoystick < Settings.joyBand && climberJoystick > -Settings.joyBand){
            climberJoystick = 0;
        }

        double armJoystick = -operatingArmXboxController.getRawAxis(1); //forward back
        if(armJoystick < Settings.joyBand && armJoystick > -Settings.joyBand){
            armJoystick = 0;
        }
        double wristJoystick = -operatingArmXboxController.getRawAxis(5); //left right
        if(wristJoystick < Settings.joyBand && wristJoystick > -Settings.joyBand){
            wristJoystick = 0;
        }

        thisRobot.arm.armMotor1.set(armJoystick);
        thisRobot.arm.armMotor2.set(-armJoystick);
        thisRobot.wrist.wristMotor1.set(wristJoystick);
        thisRobot.wrist.wristMotor2.set(-wristJoystick);
    }
    
}
