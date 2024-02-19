package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.StateMachine.robotStates;

public class TeleopController {

    Robot thisRobot;
    Joystick driverXboxController = new Joystick(Settings.driverJoystickPort);
    Joystick buttonPannel = new Joystick(Settings.buttonPannelPort);
    Joystick manualControlJoystick = new Joystick(Settings.manualControlPort);
    boolean manualRot = false;

    public TeleopController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void init(){
        //reset state machine, and set arm positions to where they are now
        thisRobot.stateMachine.robotState = robotStates.DRIVING;
        thisRobot.arm.setArmPosition(thisRobot.arm.getArmPosition());
        thisRobot.wrist.setWristPos(thisRobot.wrist.getWristPos());
        
        //reset heading PID
        thisRobot.drivebase.rotPidController.reset();

        //get alliance color
        thisRobot.updateAlliance();
    }

    public void periodic(){

        //if we are forcing manual control, or manual joystick presses both buttons go to manual mode
        if(Settings.manualTestingControl || 
            (manualControlJoystick.getRawButton(Settings.enableManualControlButton1) 
            && manualControlJoystick.getRawButton(Settings.enableManualControlButton2)))
        {
            manualControl();

        } else {
            //otherwise, run the state machine and update all motor powers
            thisRobot.stateMachine.updateRobotState();
            thisRobot.updateAllSubsystemMotorPower();
        }
        //always drive in tele
        driveTele();
    }

    public void driveTele(){

        //get each of the joysticks and check their deadbands
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

        //if we are on red, flip the joysticks
        if(thisRobot.onRedAlliance){
            xSpeedJoystick = -xSpeedJoystick;
            ySpeedJoystick = -ySpeedJoystick;
        }

        //cube the joystick values for smoother control
        double xInput = Math.pow(xSpeedJoystick, 3); 
        double yInput = Math.pow(ySpeedJoystick, 3);
        double rInput = Math.pow(rSpeedJoystick, 3);

        double xV = xInput * thisRobot.drivebase.swerveDrive.getMaximumVelocity();
        double yV = yInput * thisRobot.drivebase.swerveDrive.getMaximumVelocity();
        double rV = rInput * Settings.rotJoyRate;
        
        //if a is pressed, snap to face amp
        if(driverXboxController.getRawButton(Settings.snapToAmpButton)){
            manualRot = false;
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
        
        //if pressed, update heading to aim at speaker
        if(driverXboxController.getRawButton(Settings.aimAtSpeakerButton)){
            manualRot = false;
            thisRobot.drivebase.setGoalHeadingToGoal();
        }
        //if we are commanding a turn manuall, udpate goal heading to be the current pose +- joystick value * rotJoyRate
        if(rV != 0 || manualRot){
            manualRot = true;
            thisRobot.drivebase.driveOpenLoopHeading(new Translation2d(xV, yV), rV);
        } else {
            //after all that, call drive
            thisRobot.drivebase.driveClosedLoopHeading(new Translation2d(xV, yV));
        }

    }

    public void manualControl(){

        //reset arm wrist 0
        if(manualControlJoystick.getRawButton(Settings.manualResetZeroButton)){
            thisRobot.arm.armMotor1.getEncoder().setPosition(Settings.armInitRawEncoderValue);
            thisRobot.wrist.wristMotor1.getEncoder().setPosition(Settings.wristInitRawEncoderValue);
        }

        //get arm and wrist power apply deadband
        double armJoystick = -manualControlJoystick.getRawAxis(Settings.manualControlArmAxis) * 0.2; 
        if(armJoystick < Settings.joystickDeadband && armJoystick > -Settings.joystickDeadband){
            armJoystick = 0;
        }

        double wristJoystick = -manualControlJoystick.getRawAxis(Settings.manualControlWristAxis) * 0.2;
        if(wristJoystick < Settings.joystickDeadband && wristJoystick > -Settings.joystickDeadband){
            wristJoystick = 0;
        }

        //manually update all voltages, move arm wrist
        thisRobot.arm.armMotor1.set(armJoystick);
        thisRobot.arm.armMotor2.set(-armJoystick);
        thisRobot.wrist.wristMotor1.set(wristJoystick);
        thisRobot.wrist.wristMotor2.set(-wristJoystick);
        thisRobot.shooter.setShooterSpeeds(0);
        thisRobot.shooter.setFeederVoltage(0);
        thisRobot.intake.setIntakeVoltage(0);
        thisRobot.climber.climberMotor1.set(0);
        thisRobot.climber.climberMotor2.set(0);
    } 
}
