package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class AutoController {

    Robot thisRobot;
    double trajStartTime = -1;
    PathPlannerPath path;
    PIDController headingPidController;
    PathPlannerTrajectory autoTraj;
    int pathNum = 0;


    public AutoController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void autoInit(){
        headingPidController  = new PIDController(Settings.hc_P, Settings.hc_I, Settings.hc_D);

        path = PathPlannerPath.fromPathFile("SubWooferToFarNote");
        Pose2d initPose = path.getPreviewStartingHolonomicPose();
        thisRobot.drivebase.swerveDrive.resetOdometry(initPose);
        trajStartTime = Timer.getFPGATimestamp();
        autoTraj =  path.getTrajectory(thisRobot.drivebase.swerveDrive.getFieldVelocity(), thisRobot.drivebase.swerveDrive.getOdometryHeading());
        pathNum = 0;
    }

    public void autoPeriodic(){
        boolean runpath = true;
        if(runpath){
            //runs a path
            double trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
            State goalState = autoTraj.sample(trajElapsedTime);
            Rotation2d currentHeading = thisRobot.drivebase.swerveDrive.field.getRobotPose().getRotation();
            Rotation2d goalHeading = goalState.targetHolonomicRotation;
            
            double rotCorrectionOutput = headingPidController.calculate(currentHeading.getDegrees(), goalHeading.getDegrees());
            thisRobot.drivebase.swerveDrive.drive( new Translation2d(goalState.velocityMps, goalState.heading), rotCorrectionOutput,true,  false);
            
            if(trajElapsedTime > autoTraj.getTotalTimeSeconds() && pathNum < 1){
                path = PathPlannerPath.fromPathFile("FarNoteToFirstNoteAndBack");
                autoTraj =  path.getTrajectory(thisRobot.drivebase.swerveDrive.getFieldVelocity(), thisRobot.drivebase.swerveDrive.getOdometryHeading());
                pathNum++;
                trajStartTime = Timer.getFPGATimestamp();
            }


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
