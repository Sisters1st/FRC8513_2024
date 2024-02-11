package frc.robot.Logic;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.StateMachine.robotStates;

public class TeleopController {

    Robot thisRobot;
    Joystick driverXboxController = new Joystick(Settings.driverJoystickPort);
    Joystick operatingArmXboxController = new Joystick(Settings.opperatingArmJoystickPort);
    Joystick climberXboxController = new Joystick(Settings.climberJoystickPort);

    public TeleopController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void init(){
        thisRobot.stateMachine.robotState = robotStates.DRIVING;
        thisRobot.arm.setArmPosition(thisRobot.arm.getArmPosition());
        thisRobot.wrist.setWristPos(thisRobot.wrist.getWristPos());
        
        thisRobot.drivebase.rotPidController.reset();

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                thisRobot.onRedAlliance = true;
            }
            if (ally.get() == Alliance.Blue) {
                thisRobot.onRedAlliance = false;
            }
        }
    }

    public void periodic(){
        boolean manualTestingControl = false;
        if(manualTestingControl){
            manualControl();

        } else {
            thisRobot.stateMachine.updateRobotState();
            thisRobot.updateAllSubsystemMotorPower();
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
        double rV = rSpeedJoystick;
        thisRobot.drivebase.goalHeading = thisRobot.drivebase.goalHeading.plus(new Rotation2d(rV * Settings.rotJoyRate));
        
        if(driverXboxController.getRawButton(5)){
            thisRobot.drivebase.setGoalHeadingDeg(0);
        }
        if(driverXboxController.getRawButton(6)){
            thisRobot.drivebase.setGoalHeadingDeg(180);
        }
        if(driverXboxController.getRawButton(1)){
            thisRobot.drivebase.setGoalHeadingDeg(90);
        }
        if(driverXboxController.getRawButton(8)){
            thisRobot.drivebase.swerveDrive.resetOdometry(new Pose2d(thisRobot.drivebase.swerveDrive.getPose().getTranslation(), new Rotation2d()));
            thisRobot.drivebase.goalHeading = new Rotation2d(0);
        }
        if(driverXboxController.getRawButton(16)){
            if(thisRobot.onRedAlliance){
                thisRobot.drivebase.aimAtPoint(Settings.redGoalPos);
            }else{
                thisRobot.drivebase.aimAtPoint(Settings.blueGoalPos);
            }
            
        }

        thisRobot.drivebase.driveClosedLoopHeading(new Translation2d(xV, yV));
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
