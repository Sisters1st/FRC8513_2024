package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
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
  public Translation2d ajustedV = new Translation2d();
  public double rotCorrection = 0;

  PIDController xPosPidController = new PIDController(8, 0, 0);
  PIDController yPosPidController = new PIDController(8, 0, 0);
  PIDController rotPidController = new PIDController(20, 0, 0);

  public double trajStartTime;
  public State goalState = new State();
  double trajElapsedTime;
  Trajectory.State trajState;
  public Rotation2d goalHeading = new Rotation2d();
  ChassisSpeeds adjustedSpeeds;

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
    swerveDrive.setCosineCompensator(false);
    rotPidController.enableContinuousInput(-Math.PI,Math.PI);

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
    
      goalState = new State();
      goalHeading = new Rotation2d();
      path = PathPlannerPath.fromPathFile(pathName);

      if(flipToRed){
        path = path.flipPath();
      }

      autoTraj = path.getTrajectory(new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());

      xPosPidController.reset();
      yPosPidController.reset();
      rotPidController.reset();

      goalState = autoTraj.sample(0);
      goalHeading = goalState.targetHolonomicRotation;
      
      setOdomToPathInit();    
      trajStartTime = Timer.getFPGATimestamp();

  }

  public void followPath(){
    trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
    goalState = autoTraj.sample(trajElapsedTime);
    goalHeading = goalState.targetHolonomicRotation;

    double currentHeading = swerveDrive.getPose().getRotation().getRadians();
    double xCorrection = xPosPidController.calculate(swerveDrive.getPose().getX(), goalState.getTargetHolonomicPose().getX());
    double yCorrection = yPosPidController.calculate(swerveDrive.getPose().getY(), goalState.getTargetHolonomicPose().getY());
    
    rotCorrection = rotPidController.calculate(currentHeading, goalHeading.getRadians());
    rotCorrection = Math.min(Math.max(rotCorrection, -swerveDrive.getMaximumAngularVelocity()), swerveDrive.getMaximumAngularVelocity());
    
    Translation2d trajV = new Translation2d(goalState.velocityMps, goalHeading);
    ajustedV = new Translation2d(xCorrection, yCorrection);
    ajustedV = ajustedV.plus(trajV);
    
    thisRobot.drivebase.swerveDrive.drive(ajustedV,
    rotCorrection, true, false);
  }

  public boolean isPathOver(){
    double trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
    return trajElapsedTime > autoTraj.getTotalTimeSeconds();
  }

  public void setOdomToPathInit(){
    Pose2d initPose = goalState.getTargetHolonomicPose();
    thisRobot.drivebase.swerveDrive.resetOdometry(initPose);
  }

  public void simulateDrivebaseInit(){
  }

  public void simulateDrivebase(){
  }
}


