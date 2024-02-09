package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drivebase {

  public Robot thisRobot;
  public SwerveDrive swerveDrive;

  PathPlannerPath path;
  PathPlannerTrajectory autoTraj;
  HolonomicDriveController holonomicDriveController;
  double trajStartTime;
  public State goalState;

  public Drivebase(Robot thisRobot_) {
    thisRobot = thisRobot_;
    //yagsl init
    double maximumSpeed = Units.feetToMeters(Settings.drivebaseMaxVelocity);    
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (IOException e) {
      e.printStackTrace();
    }
    SwerveDriveTelemetry.verbosity = Settings.telemetryVerbosity;

    //pathFollowingInit
    goalState = new State();


  }

  public void updateOdometry() {
    
    if(Settings.useLimelight){
      
      double tl = LimelightHelpers.getLatency_Pipeline(Settings.limelightName);
      double cl = LimelightHelpers.getLatency_Capture(Settings.limelightName);
      double visionTime = Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0);
      //see https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
      swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPose2d(Settings.limelightName),visionTime);
    }
    
  }

  public void initPath(String pathName, boolean flipToRed){
      holonomicDriveController = new HolonomicDriveController(
        new PIDController(20, 0.5, 0), new PIDController(20, 0.1, 0),
        new ProfiledPIDController(5, 0, 0,
            new TrapezoidProfile.Constraints(thisRobot.drivebase.swerveDrive.getMaximumAngularVelocity(), 6.28)));

      path = PathPlannerPath.fromPathFile(pathName);
      if(flipToRed){
        path = path.flipPath();
      }

      Pose2d initPose = path.getPreviewStartingHolonomicPose();
      thisRobot.drivebase.swerveDrive.resetOdometry(initPose);
      trajStartTime = Timer.getFPGATimestamp();
      autoTraj =  path.getTrajectory(thisRobot.drivebase.swerveDrive.getFieldVelocity(), thisRobot.drivebase.swerveDrive.getOdometryHeading());

  }

  public void followPath(){
    double trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
    goalState = autoTraj.sample(trajElapsedTime);
    Rotation2d goalHeading = goalState.targetHolonomicRotation;
    
    Trajectory.State trajState = new Trajectory.State(goalState.timeSeconds,
        goalState.velocityMps,
        goalState.accelerationMpsSq,
        new Pose2d(goalState.positionMeters,goalHeading),
    goalState.curvatureRadPerMeter);

    ChassisSpeeds adjustedSpeeds = holonomicDriveController.calculate(
    thisRobot.drivebase.swerveDrive.getPose(), trajState, goalHeading);
    
    thisRobot.drivebase.swerveDrive.drive(adjustedSpeeds);
  }

  public boolean isPathOver(){
    double trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
    return trajElapsedTime > autoTraj.getTotalTimeSeconds();
  }

  public void simulateDrivebaseInit(){
  }

  public void simulateDrivebase(){
  }
}


