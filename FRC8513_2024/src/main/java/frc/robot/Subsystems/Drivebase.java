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

  PIDController xPosPidController = new PIDController(10, 0, 0);
  PIDController yPosPidController = new PIDController(10, 0, 0);
  PIDController rotPidController = new PIDController(0.5, 0, 0);

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
    rotPidController.enableContinuousInput(-180,180);

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
      path = PathPlannerPath.fromPathFile(pathName);

      if(flipToRed){
        path = path.flipPath();
      }

      setOdomToPathInit();
      trajStartTime = Timer.getFPGATimestamp();
      autoTraj = path.getTrajectory(thisRobot.drivebase.swerveDrive.getFieldVelocity(), thisRobot.drivebase.swerveDrive.getYaw());
      xPosPidController.reset();
      yPosPidController.reset();
      rotPidController.reset();
      goalState = autoTraj.sample(0);
      

  }

  public void followPath(){
    trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
    goalState = autoTraj.sample(trajElapsedTime);
    goalHeading = goalState.targetHolonomicRotation;

    double xCorrection = xPosPidController.calculate(swerveDrive.getPose().getX(), goalState.getTargetHolonomicPose().getX());
    double yCorrection = yPosPidController.calculate(swerveDrive.getPose().getY(), goalState.getTargetHolonomicPose().getY());
    double rotCorrection = rotPidController.calculate(swerveDrive.getPose().getRotation().getDegrees(), goalHeading.getDegrees());
    Translation2d trajV = new Translation2d(goalState.velocityMps, goalHeading);
    Translation2d ajustedV = new Translation2d(xCorrection, yCorrection);

    ajustedV = ajustedV.plus(trajV);
    
    thisRobot.drivebase.swerveDrive.drive(ajustedV,
    rotCorrection, true, false);
  }

  public boolean isPathOver(){
    double trajElapsedTime = Timer.getFPGATimestamp()  - trajStartTime;
    return trajElapsedTime > autoTraj.getTotalTimeSeconds();
  }

  public void setOdomToPathInit(){
    Pose2d initPose = path.getPreviewStartingHolonomicPose();
    if(thisRobot.onRedAlliance){
      //initPose.rotateBy(new Rotation2d(180));
    }
    thisRobot.drivebase.swerveDrive.resetOdometry(initPose);
  }

  public void simulateDrivebaseInit(){
  }

  public void simulateDrivebase(){
  }
}


