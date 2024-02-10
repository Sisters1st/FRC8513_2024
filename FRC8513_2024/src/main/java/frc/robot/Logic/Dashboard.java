package frc.robot.Logic;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard {
    int dashbordNumber = 0;
    Robot thisRobot;
    public Dashboard(Robot thisRobot_, int dashbordNumber_){
        dashbordNumber = dashbordNumber_;
        thisRobot = thisRobot_;
    }

    public void updateDashboard(){
        if(dashbordNumber >= 0){
            //general robot updates
            SmartDashboard.putNumber("Uptime", Timer.getFPGATimestamp());
            SmartDashboard.putString("State", thisRobot.stateMachine.robotState.toString());

            //auto infor
            SmartDashboard.putString("AutoRoutine", thisRobot.autoController.autoRoutine.toString());
            SmartDashboard.putNumber("AutoStep", thisRobot.autoController.autoStep);

            //drivebase info
            SmartDashboard.putNumber("trajGoalX", thisRobot.drivebase.goalState.positionMeters.getX());
            SmartDashboard.putNumber("trajGoalY", thisRobot.drivebase.goalState.positionMeters.getY());
            SmartDashboard.putNumber("Goalheading", thisRobot.drivebase.goalHeading.getDegrees());
            
            //armInfo
            SmartDashboard.putNumber("ArmPos", thisRobot.arm.armPos);
            SmartDashboard.putNumber("ArmGoalPos", thisRobot.arm.armGoalPos);
            SmartDashboard.putNumber("calculatedArmGoal", thisRobot.arm.calculatedArmGoal);


            //wristInfo
            SmartDashboard.putNumber("WristPos", thisRobot.wrist.wristPos);
            SmartDashboard.putNumber("WristGoalPos", thisRobot.wrist.wristGoalPos);
            SmartDashboard.putNumber("calculatedWristmGoal", thisRobot.wrist.calculatedWristGoal);

            //shooter infor
            SmartDashboard.putNumber("ShooterSpeed", thisRobot.shooter.leftShooter.getEncoder().getVelocity());
            SmartDashboard.putNumber("ShooterGoalSpeed", thisRobot.shooter.leftShooterGoalSpeed);
            SmartDashboard.putNumber("FeederOutput", thisRobot.shooter.feederMotor.getAppliedOutput());

            //intake info
            SmartDashboard.putNumber("intakePower", thisRobot.intake.leftIntakeMotor.getAppliedOutput());

            //climber info
            SmartDashboard.putNumber("climberPower", thisRobot.climber.climberMotor1.getAppliedOutput());
            
        }
            
    }
}
