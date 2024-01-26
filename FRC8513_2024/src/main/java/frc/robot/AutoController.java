package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoController {

    Robot thisRobot;
    double trajStartTime = -1;
    PathPlannerPath path;
    PIDController headingPidController;

    public AutoController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void autoInit(){
        path = PathPlannerPath.fromPathFile("TestPath");
        Pose2d initPose = path.getPreviewStartingHolonomicPose();
        thisRobot.drivebase.swerveDrive.resetOdometry(initPose);
        headingPidController  = new PIDController(Settings.hc_P, Settings.hc_I, Settings.hc_D);
        trajStartTime = Timer.getFPGATimestamp();

    }

    public void autoPeriodic(){
        boolean runpath = true;
        if(runpath){
            //runs a path
            PathPlannerTrajectory autoTraj =  path.getTrajectory(thisRobot.drivebase.swerveDrive.getFieldVelocity(), thisRobot.drivebase.swerveDrive.getOdometryHeading());
            double trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
            State goalState = autoTraj.sample(trajElapsedTime);
            Rotation2d currentHeading = thisRobot.drivebase.swerveDrive.field.getRobotPose().getRotation();
            Rotation2d goalHeading = goalState.targetHolonomicRotation;

            double rotCorrectionOutput = headingPidController.calculate(currentHeading.getDegrees(), goalHeading.getDegrees());
            SmartDashboard.putNumber("rotCoroutput", rotCorrectionOutput);
            SmartDashboard.putNumber("rotError", goalHeading.getDegrees() - currentHeading.getDegrees());
            SmartDashboard.putNumber("goalHeading", goalHeading.getDegrees());
            thisRobot.drivebase.swerveDrive.drive(
                new Translation2d(goalState.velocityMps, goalState.heading), 
                rotCorrectionOutput,
                true, 
                false);
        } else {
            //drive manually
            thisRobot.drivebase.swerveDrive.drive(
            new Translation2d(0.3, 0.3),
            0,
            true,
            false
        );
        }
        

    }
}
