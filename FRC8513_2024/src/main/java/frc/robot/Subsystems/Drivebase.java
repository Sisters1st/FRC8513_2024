package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;

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

  public Drivebase(Robot thisRobot_) {
    double maximumSpeed = Units.feetToMeters(Settings.drivebaseMaxVelocity);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (IOException e) {
      e.printStackTrace();
    }

    SwerveDriveTelemetry.verbosity = Settings.telemetryVerbosity;

  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    
    if(Settings.useLimelight){
      
      double tl = LimelightHelpers.getLatency_Pipeline(Settings.limelightName);
      double cl = LimelightHelpers.getLatency_Capture(Settings.limelightName);
      double visionTime = Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0);
      //see https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
      swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPose2d(Settings.limelightName),visionTime);
    }
    
  }

  public void simulateDrivebaseInit(){
  }

  public void simulateDrivebase(){
  }
}


