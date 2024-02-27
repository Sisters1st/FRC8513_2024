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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.AutoController.autoRoutines;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drivebase {

  public Robot thisRobot;
  public SwerveDrive swerveDrive;

  //path planner init
  public PathPlannerPath path;
  public PathPlannerTrajectory autoTraj;
  public Translation2d ajustedV = new Translation2d();
  public double rotCorrection = 0;

  //path PID loops
  PIDController xPosPidController = new PIDController(Settings.drivebase_PID_P, Settings.drivebase_PID_I, Settings.drivebase_PID_D);
  PIDController yPosPidController = new PIDController(Settings.drivebase_PID_P, Settings.drivebase_PID_I, Settings.drivebase_PID_D);
  public PIDController rotPidController = new PIDController(5, 0, 0.5);

  //path planning vars
  public double trajStartTime;
  public State goalState = new State();
  public double trajElapsedTime;
  public Trajectory.State trajState;
  public Rotation2d goalHeading = new Rotation2d();
  public ChassisSpeeds adjustedSpeeds;
  public PhotonCamera camera = new PhotonCamera(Settings.photonName);
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public double lastPhotonUpdateTime = 0;

  //transformation from robot to camera
  Transform3d robotToCam = new Transform3d(new Translation3d(-.27, 0, 0.35), new Rotation3d(0, .47, Math.PI));
  PhotonPoseEstimator photonPoseEstimatorOne = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
  PhotonPoseEstimator photonPoseEstimatorTwo = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

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
  }

  //use vision
  public void updateOdometry() {
    if(Settings.usePhoton){
      Optional<EstimatedRobotPose> pose2Tag = photonPoseEstimatorTwo.update();
      if (pose2Tag.isPresent()) {
        //multi pose
        
        
        swerveDrive.addVisionMeasurement(pose2Tag.get().estimatedPose.toPose2d(), pose2Tag.get().timestampSeconds);
        lastPhotonUpdateTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("2tagX", pose2Tag.get().estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("2tagY", pose2Tag.get().estimatedPose.toPose2d().getY());
      
      } else {
        
        Optional<EstimatedRobotPose> oneTagPose = photonPoseEstimatorOne.update();
        if(oneTagPose.isPresent() && Settings.useSingleTag){
          //only one target
          swerveDrive.addVisionMeasurement(oneTagPose.get().estimatedPose.toPose2d(), oneTagPose.get().timestampSeconds);
          lastPhotonUpdateTime = Timer.getFPGATimestamp();
          SmartDashboard.putNumber("1tagX", oneTagPose.get().estimatedPose.toPose2d().getX());
          SmartDashboard.putNumber("1tagY", oneTagPose.get().estimatedPose.toPose2d().getY());
        
        } else {
          //no vision
        }
      }
    }
  }

  //load a path from file
  public void initPath(String pathName, boolean flipToRed) {

    path = PathPlannerPath.fromPathFile(pathName);

    if (flipToRed) {
      path = path.flipPath();
    }

    //geneate trajectory from path
    autoTraj = path.getTrajectory(new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());

    xPosPidController.reset();
    yPosPidController.reset();
    rotPidController.reset();

    goalState = autoTraj.sample(0);
    goalHeading = goalState.targetHolonomicRotation;

    // we need to test, when do we want to force odom, when no vision???
    if(Robot.isSimulation() || thisRobot.autoController.autoRoutine == autoRoutines._XTESTINGACCURACY){
      setOdomToPathInit();
    }
    
    trajStartTime = Timer.getFPGATimestamp();

  }

  //apply control to follow each step of path
  public void followPath() {
    //get current sample
    trajElapsedTime = Timer.getFPGATimestamp() - trajStartTime;
    goalState = autoTraj.sample(trajElapsedTime);
    goalHeading = goalState.targetHolonomicRotation;

    //update xy PID controllers
    double xCorrection = xPosPidController.calculate(swerveDrive.getPose().getX(),
        goalState.getTargetHolonomicPose().getX());
    double yCorrection = yPosPidController.calculate(swerveDrive.getPose().getY(),
        goalState.getTargetHolonomicPose().getY());

    setGoalHeadingDeg(goalHeading.getDegrees());

    //add the path velocity with the PID velocity
    Translation2d trajV = new Translation2d(goalState.velocityMps, goalHeading);
    ajustedV = new Translation2d(xCorrection, yCorrection);
    ajustedV = ajustedV.plus(trajV);

    //drive at that velocity
    driveClosedLoopHeading(ajustedV);
  }

  public boolean isPathOver() {
    double trajElapsedTime = Timer.getFPGATimestamp() - trajStartTime;
    return trajElapsedTime > autoTraj.getTotalTimeSeconds();
  }

  public void setOdomToPathInit() {
    Pose2d initPose = goalState.getTargetHolonomicPose();
    thisRobot.drivebase.swerveDrive.resetOdometry(initPose);
  }

  //drive with closed loop heading control
  public void driveClosedLoopHeading(Translation2d translation) {

    double rot = rotPidController.calculate(swerveDrive.getOdometryHeading().getRadians(),
        thisRobot.drivebase.goalHeading.getRadians());

    thisRobot.drivebase.swerveDrive.drive(
        translation,
        rot,
        true,
        false);
  }

  public void driveOpenLoopHeading(Translation2d translation, double rV){
    
    goalHeading = swerveDrive.getOdometryHeading();
    thisRobot.drivebase.swerveDrive.drive(
        translation,
        rV,
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

  public boolean inHeadingThold(){
    return Math.abs(swerveDrive.getPose().getRotation().minus(goalHeading).getDegrees()) < Settings.headingThold;
  }

  public boolean inVThold(){
    double xv = swerveDrive.getRobotVelocity().vxMetersPerSecond;
    double yv = swerveDrive.getRobotVelocity().vyMetersPerSecond;
    return Math.sqrt(xv * xv + yv * yv) < Settings.maxShotSpeed;
  }

  public boolean visionIsRecent(){
    return Timer.getFPGATimestamp() - lastPhotonUpdateTime < Settings.stalePhotonTime;
  }

  public void setGoalHeadingToGoal(){
    if(thisRobot.onRedAlliance){
        thisRobot.drivebase.aimAtPoint(Settings.redGoalPos);
    }else{
        thisRobot.drivebase.aimAtPoint(Settings.blueGoalPos);
    }
  }
  
  public void aimAtGoal(){
    setGoalHeadingToGoal();
    driveClosedLoopHeading(new Translation2d());
  }

}
