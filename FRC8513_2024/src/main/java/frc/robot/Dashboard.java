package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
            
            //armInfo
            SmartDashboard.putNumber("ArmRawEncoder", thisRobot.arm.armMotor1.getEncoder().getPosition());
            SmartDashboard.putNumber("ArmPosition", thisRobot.arm.armAngle);

            //wristInfo
            SmartDashboard.putNumber("WristRawPosition", thisRobot.wrist.wristMotor1.getEncoder().getPosition());
            SmartDashboard.putNumber("WristToArmAngel", thisRobot.wrist.wristAngleToArm);
            SmartDashboard.putNumber("WristToGroundAngel", thisRobot.wrist.wristAngleToGround);

            //shooter infor
            
            SmartDashboard.putNumber("ShooterSpeed", thisRobot.shooter.leftShooter.getEncoder().getVelocity());
            SmartDashboard.putNumber("ShooterGoalSpeed", thisRobot.shooter.leftShooterGoalSpeed);
          
            
        }
            
    }
}
