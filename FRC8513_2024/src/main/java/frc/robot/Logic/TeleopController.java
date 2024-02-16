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
    Joystick buttonPannel = new Joystick(Settings.buttonPannelPort);
    Joystick manualControlJoystick = new Joystick(Settings.manualControlPort);

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
        if(Settings.manualTestingControl || 
            (manualControlJoystick.getRawButton(Settings.enableManualControlButton1) 
            && manualControlJoystick.getRawButton(Settings.enableManualControlButton2)))
        {
            manualControl();

        } else {
            thisRobot.stateMachine.updateRobotState();
            thisRobot.updateAllSubsystemMotorPower();
        }
        
        driveTele();
    }

    public void driveTele(){
        double xSpeedJoystick = -driverXboxController.getRawAxis(Settings.forwardBackwardsAxis); //forward back
        if(xSpeedJoystick < Settings.joystickDeadband && xSpeedJoystick > -Settings.joystickDeadband){
            xSpeedJoystick = 0;
        }

        double ySpeedJoystick = -driverXboxController.getRawAxis(Settings.leftRightAxis); //left right
         if(ySpeedJoystick < Settings.joystickDeadband && ySpeedJoystick > -Settings.joystickDeadband){
            ySpeedJoystick = 0;
        }
        double rSpeedJoystick = -driverXboxController.getRawAxis(Settings.rotAxis); //left right 2 at home, 4 on xbox
         if(rSpeedJoystick < Settings.joystickDeadband && rSpeedJoystick > -Settings.joystickDeadband){
            rSpeedJoystick = 0;
        }

        if(thisRobot.onRedAlliance){
            xSpeedJoystick = -xSpeedJoystick;
            ySpeedJoystick = -ySpeedJoystick;
        }

        double xInput = Math.pow(xSpeedJoystick, 3); // Smooth controll out
        double yInput = Math.pow(ySpeedJoystick, 3); // Smooth controll out

        double xV = xInput * thisRobot.drivebase.swerveDrive.getMaximumVelocity();
        double yV = yInput * thisRobot.drivebase.swerveDrive.getMaximumVelocity();
        double rV = rSpeedJoystick;
        thisRobot.drivebase.goalHeading = thisRobot.drivebase.swerveDrive.getPose().getRotation().plus(new Rotation2d(rV * Settings.rotJoyRate));
        
        if(driverXboxController.getRawButton(Settings.snapToSpeakerButton)){
            if(thisRobot.onRedAlliance){
                thisRobot.drivebase.setGoalHeadingDeg(180);
            } else {
                thisRobot.drivebase.setGoalHeadingDeg(0);
            }
        }

        if(driverXboxController.getRawButton(Settings.snapToAmpButton)){
            thisRobot.drivebase.setGoalHeadingDeg(90);
        }

        //if button 8 is pressed, reset the heading
        if(driverXboxController.getRawButton(Settings.resetFieldCentricButton)){
            //if we are red, set to 180 degrees (towards blue) as the zero 
          if(thisRobot.onRedAlliance){
                thisRobot.drivebase.swerveDrive.resetOdometry(new Pose2d(thisRobot.drivebase.swerveDrive.getPose().getTranslation(), new Rotation2d(Math.PI)));
                thisRobot.drivebase.goalHeading = new Rotation2d(Math.PI);
            } else {
                thisRobot.drivebase.swerveDrive.resetOdometry(new Pose2d(thisRobot.drivebase.swerveDrive.getPose().getTranslation(), new Rotation2d()));
                thisRobot.drivebase.goalHeading = new Rotation2d(0);
            }
        }
        
        if(driverXboxController.getRawButton(Settings.aimAtSpeakerButton)){
            if(thisRobot.onRedAlliance){
                thisRobot.drivebase.aimAtPoint(Settings.redGoalPos);
            }else{
                thisRobot.drivebase.aimAtPoint(Settings.blueGoalPos);
            }
        }

        //Settings.usePhoton = driverXboxController.getRawAxis(3)>0.1;

        thisRobot.drivebase.driveClosedLoopHeading(new Translation2d(xV, yV));
    }

    //needs to be tested on robot
    public void manualControl(){

        if(manualControlJoystick.getRawButton(Settings.manualResetZeroButton)){
            thisRobot.arm.armMotor1.getEncoder().setPosition(Settings.armInitRawEncoderValue);
            thisRobot.wrist.wristMotor1.getEncoder().setPosition(Settings.wristInitRawEncoderValue);
        }

        double armJoystick = -manualControlJoystick.getRawAxis(Settings.manualControlArmAxis); //forward back
        if(armJoystick < Settings.joystickDeadband && armJoystick > -Settings.joystickDeadband){
            armJoystick = 0;
        }

        double wristJoystick = -manualControlJoystick.getRawAxis(Settings.manualControlWristAxis); //left right
        if(wristJoystick < Settings.joystickDeadband && wristJoystick > -Settings.joystickDeadband){
            wristJoystick = 0;
        }

        thisRobot.arm.armMotor1.set(armJoystick);
        thisRobot.arm.armMotor2.set(-armJoystick);
        thisRobot.wrist.wristMotor1.set(wristJoystick);
        thisRobot.wrist.wristMotor2.set(-wristJoystick);
        thisRobot.shooter.setShooterSpeeds(0,0,0);
        thisRobot.intake.setIntakeVoltage(0);
        thisRobot.climber.climberMotor1.set(0);
        thisRobot.climber.climberMotor2.set(0);
    } 
}
