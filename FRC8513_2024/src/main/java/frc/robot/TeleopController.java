package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class TeleopController {

    Robot thisRobot;

    XboxController driverXboxController = new XboxController(Settings.driverJoystickPort);

    boolean fieldCentric = Settings.teleopFieldCentricDefault;

    public TeleopController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void init(){

    }

    public void periodic(){

        double xSpeed = driverXboxController.getLeftY() * Settings.maxDBSpeed;
        double ySpeed = driverXboxController.getLeftX() * Settings.maxDBSpeed;

        double rotSpeed = -driverXboxController.getRightX() * Settings.maxDBAngularSpeed;

        if(driverXboxController.getRawButtonPressed(Settings.driver_fieldCentricButton)){
            fieldCentric = !fieldCentric;
            thisRobot.drivebase.m_gyro.reset();
        }

        thisRobot.drivebase.drive(xSpeed, ySpeed, rotSpeed, fieldCentric, thisRobot.getPeriod());
    }
    
}
