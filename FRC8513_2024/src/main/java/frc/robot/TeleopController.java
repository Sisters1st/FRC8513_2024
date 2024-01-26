package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

public class TeleopController {

    Robot thisRobot;
    Joystick driverXboxController = new Joystick(Settings.driverJoystickPort);

    public TeleopController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void init(){

    }

    public void periodic(){

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

    
}
