package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  public PathPlannerPath path;
  public PathPlannerTrajectory autoTraj;
  public Translation2d ajustedV = new Translation2d();
  public double rotCorrection = 0;

  PIDController xPosPidController = new PIDController(8, 0, 0);
  PIDController yPosPidController = new PIDController(8, 0, 0);
  public PIDController rotPidController = new PIDController(5, 0, 0);

  public double trajStartTime;
  public State goalState = new State();
  public double trajElapsedTime;
  public Trajectory.State trajState;
  public Rotation2d goalHeading = new Rotation2d();
  public ChassisSpeeds adjustedSpeeds;
  public PhotonCamera camera = new PhotonCamera(Settings.photonName);
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  PhotonPoseEstimator photonPoseEstimator;

  Transform3d robotToCam = new Transform3d(new Translation3d(-.27, 0, 0.35), new Rotation3d(0, .47, Math.PI));

  public Drivebase(Robot thisRobot_) {
    thisRobot = thisRobot_;

    // yagsl init
    double maximumSpeed = Units.feetToMeters(Settings.drivebaseMaxVelocity);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (IOException e) {
      e.printStackTrace();
    }
    SwerveDriveTelemetry.verbosity = Settings.telemetryVerbosity;
    swerveDrive.setCosineCompensator(false);
    rotPidController.enableContinuousInput(-Math.PI, Math.PI);

    // pathFollowingInit
    goalState = new State();

    // photon init
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera, robotToCam);
  }

  public void updateOdometry() {
    if(Settings.usePhoton){
      var result = camera.getLatestResult();
      if (result.getMultiTagResult().estimatedPose.isPresent) {
        Optional<EstimatedRobotPose> photonPose = photonPoseEstimator.update(result);
        //System.out.println("GotMultiTagPNP");
        if(photonPose.isPresent()){
          //System.out.println("posePresent");
          swerveDrive.addVisionMeasurement(photonPose.get().estimatedPose.toPose2d(), result.getTimestampSeconds());
        }
      }
    }
  }

  public void initPath(String pathName, boolean flipToRed) {

    path = PathPlannerPath.fromPathFile(pathName);

    if (flipToRed) {
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

  public void followPath() {
    trajElapsedTime = Timer.getFPGATimestamp() - trajStartTime;
    goalState = autoTraj.sample(trajElapsedTime);
    goalHeading = goalState.targetHolonomicRotation;

    double currentHeading = swerveDrive.getPose().getRotation().getRadians();
    double xCorrection = xPosPidController.calculate(swerveDrive.getPose().getX(),
        goalState.getTargetHolonomicPose().getX());
    double yCorrection = yPosPidController.calculate(swerveDrive.getPose().getY(),
        goalState.getTargetHolonomicPose().getY());

    rotCorrection = rotPidController.calculate(currentHeading, goalHeading.getRadians());
    rotCorrection = Math.min(Math.max(rotCorrection, -swerveDrive.getMaximumAngularVelocity()),
        swerveDrive.getMaximumAngularVelocity());

    Translation2d trajV = new Translation2d(goalState.velocityMps, goalHeading);
    ajustedV = new Translation2d(xCorrection, yCorrection);
    ajustedV = ajustedV.plus(trajV);

    thisRobot.drivebase.swerveDrive.drive(ajustedV, rotCorrection, true, false);
  }

  public boolean isPathOver() {
    double trajElapsedTime = Timer.getFPGATimestamp() - trajStartTime;
    return trajElapsedTime > autoTraj.getTotalTimeSeconds();
  }

  public void setOdomToPathInit() {
    Pose2d initPose = goalState.getTargetHolonomicPose();
    thisRobot.drivebase.swerveDrive.resetOdometry(initPose);
  }

  public void driveClosedLoopHeading(Translation2d translation) {

    double rot = rotPidController.calculate(swerveDrive.getOdometryHeading().getRadians(),
        thisRobot.drivebase.goalHeading.getRadians());

    thisRobot.drivebase.swerveDrive.drive(
        translation,
        rot,
        true,
        false);
  }

  public void setGoalHeadingDeg(double deg) {
    goalHeading = new Rotation2d(Math.toRadians(deg));
  }

  public void aimAtPoint(Translation2d point) {
    Translation2d currentPos = swerveDrive.getPose().getTranslation();
    Translation2d deltaPos = currentPos.minus(point);
    goalHeading = deltaPos.getAngle();
  }

}
