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
            SmartDashboard.putNumber("maxAng", thisRobot.drivebase.swerveDrive.getMaximumAngularVelocity());
            SmartDashboard.putNumber("currentWheelV", thisRobot.drivebase.swerveDrive.getStates()[3].speedMetersPerSecond);
            //SmartDashboard.putNumber("desiredWheelV", thisRobot.drivebase.swerveDrive..getState().speedMetersPerSecond);
            SmartDashboard.putNumber("V", thisRobot.drivebase.swerveDrive.getRobotVelocity().vxMetersPerSecond);
            


          
            
        }
            
    }
}
