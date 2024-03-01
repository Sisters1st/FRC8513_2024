package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.AutoController.autoRoutines;

public class Dashboard {
    Robot thisRobot;
    public SendableChooser<String> autoSelector;

    public Dashboard(Robot thisRobot_) {
        thisRobot = thisRobot_;

        // create auto selector with each enum option
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption(autoRoutines.values()[0].toString(), autoRoutines.values()[0].toString());
        for (int i = 1; i < autoRoutines.values().length; i++) {
            autoSelector.addOption(autoRoutines.values()[i].toString(), autoRoutines.values()[i].toString());
        }

        SmartDashboard.putData("Auton Selector", autoSelector);

    }

    public void updateDashboard() {

        // general robot updates
        SmartDashboard.putNumber("Uptime", Timer.getFPGATimestamp());
        SmartDashboard.putString("State", thisRobot.stateMachine.robotState.toString());
        SmartDashboard.putBoolean("Photon Vision Enabled", Settings.usePhoton);
        SmartDashboard.putNumber("LastPhotonUpdate", thisRobot.drivebase.lastPhotonUpdateTime);

        // auto info
        SmartDashboard.putNumber("AutoStep", thisRobot.autoController.autoStep);

        // drivebase info
        SmartDashboard.putNumber("trajGoalXP", thisRobot.drivebase.goalState.positionMeters.getX());
        SmartDashboard.putNumber("trajGoalYP", thisRobot.drivebase.goalState.positionMeters.getY());
        SmartDashboard.putNumber("Goalheading", thisRobot.drivebase.goalHeading.getDegrees());
        SmartDashboard.putBoolean("inHeadingThold", thisRobot.drivebase.inHeadingThold());

        SmartDashboard.putNumber("trajXV", thisRobot.drivebase.ajustedV.getX());
        SmartDashboard.putNumber("trajYV", thisRobot.drivebase.ajustedV.getY());
        SmartDashboard.putNumber("trajRotV", thisRobot.drivebase.rotCorrection);
        SmartDashboard.putNumber("DistToGoal", thisRobot.shooter.getDistFromGoal());

        SmartDashboard.putNumber("Roll", thisRobot.drivebase.gyro.getRoll());

        // armInfo
        SmartDashboard.putNumber("ArmPos", thisRobot.arm.getArmPosition());
        SmartDashboard.putNumber("ArmGoalPos", thisRobot.arm.armGoalPos);
        SmartDashboard.putNumber("calculatedArmGoal", thisRobot.arm.calculatedArmGoal);
        SmartDashboard.putBoolean("ArmInThold", thisRobot.arm.armWithinThold());

        // wristInfo
        SmartDashboard.putNumber("WristPos", thisRobot.wrist.getWristPos());
        SmartDashboard.putNumber("WristGoalPos", thisRobot.wrist.wristGoalPos);
        SmartDashboard.putNumber("calculatedWristGoal", thisRobot.wrist.calculatedWristGoal);
        SmartDashboard.putBoolean("WristInThold", thisRobot.wrist.wristWithinThold());
        SmartDashboard.putNumber("WristFromDist",
                thisRobot.stateMachine.getWristAngFromDist(thisRobot.shooter.getDistFromGoal()));

        // shooter infor
        SmartDashboard.putNumber("ShooterLeftSpeed", thisRobot.shooter.leftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("ShooterLeftGoalSpeed", thisRobot.shooter.leftShooterGoalSpeed);
        SmartDashboard.putNumber("ShooterRightSpeed", thisRobot.shooter.rightShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("ShooterRightGoalSpeed", thisRobot.shooter.rightShooterGoalSpeed);
        SmartDashboard.putBoolean("ShooterInThold",
                thisRobot.shooter.leftShooterInThreshold() && thisRobot.shooter.rightShooterInThreshold());

        SmartDashboard.putNumber("FeederOutput", thisRobot.shooter.feederMotor.getAppliedOutput());
        SmartDashboard.putNumber("feederSensor", thisRobot.shooter.feederSensorInput.getValue());
        SmartDashboard.putNumber("ShotWristOveride", thisRobot.wristOveride);

        // intake info
        SmartDashboard.putNumber("intakePower", thisRobot.intake.leftIntakeMotor.getAppliedOutput());

        // climber info
        SmartDashboard.putNumber("leftClimberPower", thisRobot.climber.climberMotor2.getAppliedOutput());
        SmartDashboard.putNumber("rightClimberPower", thisRobot.climber.climberMotor1.getAppliedOutput());

    }
}
