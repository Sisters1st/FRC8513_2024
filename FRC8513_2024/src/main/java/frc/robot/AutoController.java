package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoController {

    Robot thisRobot;
    double trajStartTime = -1;
    PathPlannerPath path;
    PIDController headingPidController;
    PathPlannerTrajectory autoTraj;
    int pathNum = 0;
    HolonomicDriveController holonomicDriveController;


    public AutoController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void autoInit(){
        headingPidController  = new PIDController(Settings.hc_P, Settings.hc_I, Settings.hc_D);

        holonomicDriveController = new HolonomicDriveController(
        new PIDController(20, 0.5, 0), new PIDController(20, 0.1, 0),
        new ProfiledPIDController(5, 0, 0,
            new TrapezoidProfile.Constraints(thisRobot.drivebase.swerveDrive.getMaximumAngularVelocity(), 6.28)));

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
            Rotation2d goalHeading = goalState.targetHolonomicRotation;

            SmartDashboard.putNumber("goalX", goalState.positionMeters.getX());
            
            SmartDashboard.putNumber("goalY", goalState.positionMeters.getY());
            
            Trajectory.State trajState = new Trajectory.State(goalState.timeSeconds,
                goalState.velocityMps,
                goalState.accelerationMpsSq,
                new Pose2d(goalState.positionMeters,goalHeading),
            goalState.curvatureRadPerMeter);

            ChassisSpeeds adjustedSpeeds = holonomicDriveController.calculate(
            thisRobot.drivebase.swerveDrive.getPose(), trajState, goalHeading);
            
            //thisRobot.drivebase.swerveDrive.drive( new Translation2d(goalState.velocityMps, goalState.heading), rotCorrectionOutput,true,  false);
            thisRobot.drivebase.swerveDrive.drive(adjustedSpeeds);
            if(trajElapsedTime > autoTraj.getTotalTimeSeconds() && pathNum <= 0){
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
