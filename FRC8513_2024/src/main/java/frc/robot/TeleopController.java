package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class TeleopController {

    Robot thisRobot;
    XboxController driverXboxController = new XboxController(Settings.driverJoystickPort);
    public boolean fieldCentric = true;

    public TeleopController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void init(){

    }

    public void periodic(){
        double xSpeedJoystick = driverXboxController.getLeftY(); //forward back
        double ySpeedJoystick = driverXboxController.getLeftX(); //left right
        double rSpeedJoystick = driverXboxController.getRightX(); //left right

        double xInput = Math.pow(xSpeedJoystick, 3); // Smooth controll out
        double yInput = Math.pow(ySpeedJoystick, 3); // Smooth controll out

        double xV = xInput * thisRobot.drivebase.swerveDrive.getMaximumVelocity();
        double yV = yInput * thisRobot.drivebase.swerveDrive.getMaximumVelocity();
        double rV = rSpeedJoystick * thisRobot.drivebase.swerveDrive.getMaximumAngularVelocity();

        thisRobot.drivebase.swerveDrive.drive(
            new Translation2d(xV, yV),
            rV,
            fieldCentric,
            true
        );
    }

    
}
