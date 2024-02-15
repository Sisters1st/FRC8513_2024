package frc.robot.Logic;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Logic.AutoController.autoRoutines;

public class Dashboard {
    int dashbordNumber = 0;
    Robot thisRobot;
    public SendableChooser<String> autoSelector;

    public Dashboard(Robot thisRobot_, int dashbordNumber_){
        dashbordNumber = dashbordNumber_;
        thisRobot = thisRobot_;
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption(autoRoutines.values()[0].toString(), autoRoutines.values()[0].toString());
        for(int i = 1; i < autoRoutines.values().length; i++){
            autoSelector.addOption(autoRoutines.values()[i].toString(),autoRoutines.values()[i].toString());
        }

        SmartDashboard.putData("autoSelector", autoSelector);
    }

    public void updateDashboard(){
        if(dashbordNumber >= 0){
            //general robot updates
            SmartDashboard.putNumber("Uptime", Timer.getFPGATimestamp());
            SmartDashboard.putString("State", thisRobot.stateMachine.robotState.toString());

            //auto infor
            SmartDashboard.putString("AutoRoutine", autoSelector.getSelected());
            SmartDashboard.putNumber("AutoStep", thisRobot.autoController.autoStep);

            //drivebase info
            SmartDashboard.putNumber("trajGoalX", thisRobot.drivebase.goalState.positionMeters.getX());
            SmartDashboard.putNumber("trajGoalY", thisRobot.drivebase.goalState.positionMeters.getY());
            SmartDashboard.putNumber("Goalheading", thisRobot.drivebase.goalHeading.getDegrees());

            SmartDashboard.putNumber("trajXV", thisRobot.drivebase.ajustedV.getX());
            SmartDashboard.putNumber("trajYV", thisRobot.drivebase.ajustedV.getY());
            SmartDashboard.putNumber("trajRotV", thisRobot.drivebase.rotCorrection);
            
            //armInfo
            SmartDashboard.putNumber("ArmPos", thisRobot.arm.getArmPosition());
            SmartDashboard.putNumber("ArmGoalPos", thisRobot.arm.armGoalPos);
            SmartDashboard.putNumber("calculatedArmGoal", thisRobot.arm.calculatedArmGoal);
            SmartDashboard.putBoolean("ArmInThold", thisRobot.arm.armWithinThold());


            //wristInfo
            SmartDashboard.putNumber("WristPos", thisRobot.wrist.getWristPos());
            SmartDashboard.putNumber("WristGoalPos", thisRobot.wrist.wristGoalPos);
            SmartDashboard.putNumber("calculatedWristmGoal", thisRobot.wrist.calculatedWristGoal);
            SmartDashboard.putBoolean("WristInThold", thisRobot.wrist.wristWithinThold());

            //shooter infor
            SmartDashboard.putNumber("ShooterSpeed", thisRobot.shooter.leftShooter.getEncoder().getVelocity());
            SmartDashboard.putNumber("ShooterGoalSpeed", thisRobot.shooter.leftShooterGoalSpeed);
            SmartDashboard.putNumber("FeederOutput", thisRobot.shooter.feederMotor.getAppliedOutput());
            SmartDashboard.putBoolean("ShooterInThold", thisRobot.shooter.leftShooterInThreshold() && thisRobot.shooter.rightShooterInThreshold());
            SmartDashboard.putNumber("feederSensor", thisRobot.shooter.feederSensorInput.getValue());
            //intake info
            SmartDashboard.putNumber("intakePower", thisRobot.intake.leftIntakeMotor.getAppliedOutput());

            //climber info
            SmartDashboard.putNumber("climberPower", thisRobot.climber.climberMotor1.getAppliedOutput());

            
            
        }
            
    }
}
