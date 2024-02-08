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
            
            //armInfo
            SmartDashboard.putNumber("ArmRawEncoder", thisRobot.arm.armMotor1.getEncoder().getPosition());
            SmartDashboard.putNumber("ArmPosition", thisRobot.arm.armAngle);
            SmartDashboard.putNumber("ArmGoalAngle", thisRobot.arm.armGoalAngle);


            //wristInfo
            SmartDashboard.putNumber("WristRawPosition", thisRobot.wrist.wristMotor1.getEncoder().getPosition());
            SmartDashboard.putNumber("WristToArmAngel", thisRobot.wrist.wristAngleToArm);
            SmartDashboard.putNumber("WristGoalAgnle", thisRobot.wrist.wristGoalAngleToGround);

            //shooter infor
            SmartDashboard.putNumber("ShooterSpeed", thisRobot.shooter.leftShooter.getEncoder().getVelocity());
            SmartDashboard.putNumber("ShooterGoalSpeed", thisRobot.shooter.leftShooterGoalSpeed);
            SmartDashboard.putNumber("FeederOutput", thisRobot.shooter.feederMotor.getAppliedOutput());

            //intake info
            SmartDashboard.putNumber("intakePower", thisRobot.intake.leftIntake.getAppliedOutput());

            //climber info
            SmartDashboard.putNumber("climberPoewr", thisRobot.climber.climberMotor1.getAppliedOutput());
            
        }
            
    }
}
