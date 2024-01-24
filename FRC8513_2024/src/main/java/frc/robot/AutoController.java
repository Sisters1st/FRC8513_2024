package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class AutoController {

    Robot thisRobot;
    double trajStartTime = -1;
    PathPlannerPath path;

    public AutoController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void autoInit(){
        path = PathPlannerPath.fromPathFile("SubwooferToFirstNote");
        Pose2d initPose = path.getStartingDifferentialPose();
        thisRobot.drivebase.swerveDrive.resetOdometry(initPose);

        trajStartTime = Timer.getFPGATimestamp();

    }

    public void autoPeriodic(){
        
        PathPlannerTrajectory autoTraj =  path.getTrajectory(thisRobot.drivebase.swerveDrive.getFieldVelocity(), thisRobot.drivebase.swerveDrive.getOdometryHeading());
        double trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
        State goalState = autoTraj.sample(trajElapsedTime);
        thisRobot.drivebase.swerveDrive.drive(
            new Translation2d(goalState.velocityMps, goalState.heading), 
            goalState.targetHolonomicRotation.getRadians(),
            true, 
            false);

    }
}
