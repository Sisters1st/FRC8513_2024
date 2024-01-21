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

        }
        
        if(dashbordNumber >= 1){
            SmartDashboard.putNumber("OdomX", thisRobot.drivebase.m_odometry.getEstimatedPosition().getX());
            SmartDashboard.putNumber("OdomY", thisRobot.drivebase.m_odometry.getEstimatedPosition().getY());
            SmartDashboard.putNumber("OdomTheta", thisRobot.drivebase.m_odometry.getEstimatedPosition().getRotation().getDegrees());

            SmartDashboard.putNumber("GoalSpeedX", thisRobot.drivebase.xSpeedGoal);
            SmartDashboard.putNumber("GoalSpeedY", thisRobot.drivebase.ySpeedGoal);
            SmartDashboard.putNumber("GoalSpeedRot", thisRobot.drivebase.rotSpeedGoal);

        }

        if(dashbordNumber >= 2){

            //front left
            SmartDashboard.putNumber("FrontLeftSpeed", thisRobot.drivebase.m_frontLeft.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("FrontLeftAng", thisRobot.drivebase.m_frontLeft.getState().angle.getDegrees());

            SmartDashboard.putNumber("FrontLeftGoalSpeed", thisRobot.drivebase.m_frontLeft.goalSwerveState.speedMetersPerSecond);
            SmartDashboard.putNumber("FrontLeftGoalAng", thisRobot.drivebase.m_frontLeft.goalSwerveState.angle.getDegrees());
            
            //front Right
            SmartDashboard.putNumber("FrontRightSpeed", thisRobot.drivebase.m_frontRight.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("FrontRightAng", thisRobot.drivebase.m_frontRight.getState().angle.getDegrees());

            SmartDashboard.putNumber("FrontRightGoalSpeed", thisRobot.drivebase.m_frontRight.goalSwerveState.speedMetersPerSecond);
            SmartDashboard.putNumber("FrontRightGoalAng", thisRobot.drivebase.m_frontRight.goalSwerveState.angle.getDegrees());

            //back left
            SmartDashboard.putNumber("backLeftSpeed", thisRobot.drivebase.m_backLeft.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("backLeftAng", thisRobot.drivebase.m_backLeft.getState().angle.getDegrees());

            SmartDashboard.putNumber("backLeftGoalSpeed", thisRobot.drivebase.m_backLeft.goalSwerveState.speedMetersPerSecond);
            SmartDashboard.putNumber("backLeftGoalAng", thisRobot.drivebase.m_backLeft.goalSwerveState.angle.getDegrees());

            //back Right
            SmartDashboard.putNumber("backRightSpeed", thisRobot.drivebase.m_backRight.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("backRightAng", thisRobot.drivebase.m_backRight.getState().angle.getDegrees());

            SmartDashboard.putNumber("backRightGoalSpeed", thisRobot.drivebase.m_backRight.goalSwerveState.speedMetersPerSecond);
            SmartDashboard.putNumber("backRightGoalAng", thisRobot.drivebase.m_backRight.goalSwerveState.angle.getDegrees());

        
        }
        
        if(dashbordNumber >= 3){
            //all possible values
            SmartDashboard.putNumber("rawFrontLeftTurnEncoder", thisRobot.drivebase.m_frontLeft.getTurnDegrees());
            SmartDashboard.putNumber("rawFrontRightTurnEncoder", thisRobot.drivebase.m_frontRight.getTurnDegrees());
            SmartDashboard.putNumber("rawBackRightTurnEncoder", thisRobot.drivebase.m_backRight.getTurnDegrees());
            SmartDashboard.putNumber("rawBackLeftTurnEncoder", thisRobot.drivebase.m_backLeft.getTurnDegrees());

            SmartDashboard.putNumber("rawFrontLeftTurnVel", thisRobot.drivebase.m_frontLeft.getTurnDegrees());
            SmartDashboard.putNumber("rawFrontRightTurnVel", thisRobot.drivebase.m_frontRight.getTurnDegrees());
            SmartDashboard.putNumber("rawBackRightTurnVel", thisRobot.drivebase.m_backRight.getTurnDegrees());
            SmartDashboard.putNumber("rawBackLeftTurnVel", thisRobot.drivebase.m_backLeft.getTurnDegrees());
            
            SmartDashboard.putNumber("rawFrontLeftDriveEncoder", thisRobot.drivebase.m_frontLeft.m_driveMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("rawFrontRightDriveEncoder", thisRobot.drivebase.m_frontRight.m_driveMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("rawBackRightDriveEncoder", thisRobot.drivebase.m_backRight.m_driveMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("rawBackLeftDriveEncoder", thisRobot.drivebase.m_backLeft.m_driveMotor.getEncoder().getPosition());

            SmartDashboard.putNumber("frontLeftDriveDist", thisRobot.drivebase.m_frontLeft.getPosition().distanceMeters);
            SmartDashboard.putNumber("frontRightDriveDist", thisRobot.drivebase.m_frontRight.getPosition().distanceMeters);
            SmartDashboard.putNumber("backLeftDriveDist", thisRobot.drivebase.m_backRight.getPosition().distanceMeters);
            SmartDashboard.putNumber("backRightDriveDist", thisRobot.drivebase.m_backLeft.getPosition().distanceMeters);

            //see what power is applied to the motors
            SmartDashboard.putNumber("frontRightDriveAppliedMotorVoltage",thisRobot.drivebase.m_frontRight.m_driveMotor.getAppliedOutput());
            SmartDashboard.putNumber("frontLeftDriveAppliedMotorVoltage",thisRobot.drivebase.m_frontLeft.m_driveMotor.getAppliedOutput());
            SmartDashboard.putNumber("backRightDriveAppliedMotorVoltage",thisRobot.drivebase.m_backRight.m_driveMotor.getAppliedOutput());
            SmartDashboard.putNumber("backLeftDriveAppliedMotorVoltage",thisRobot.drivebase.m_backLeft.m_driveMotor.getAppliedOutput());
            
            SmartDashboard.putNumber("frontRightTurnAppliedMotorVoltage",thisRobot.drivebase.m_frontRight.m_turningMotor.getAppliedOutput());
            SmartDashboard.putNumber("frontLeftTurnAppliedMotorVoltage",thisRobot.drivebase.m_frontLeft.m_turningMotor.getAppliedOutput());
            SmartDashboard.putNumber("backRightTurnAppliedMotorVoltage",thisRobot.drivebase.m_backRight.m_turningMotor.getAppliedOutput());
            SmartDashboard.putNumber("backLeftTurnAppliedMotorVoltage",thisRobot.drivebase.m_backLeft.m_turningMotor.getAppliedOutput());

            
        }
            
    }
}
