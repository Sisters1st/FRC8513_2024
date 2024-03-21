package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.Logic.LimelightHelpers;
import frc.robot.Logic.LinearInterp;
import frc.robot.Logic.AutoController.autoRoutines;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drivebase {

  public Robot thisRobot;
  public SwerveDrive swerveDrive;

  // path planner init
  public PathPlannerPath path;
  public PathPlannerTrajectory autoTraj;
  public Translation2d ajustedV = new Translation2d();
  public double rotCorrection = 0;

  // path PID loops
  PIDController xPosPidController = new PIDController(Settings.drivebase_PID_P, Settings.drivebase_PID_I,
      Settings.drivebase_PID_D);
  PIDController yPosPidController = new PIDController(Settings.drivebase_PID_P, Settings.drivebase_PID_I,
      Settings.drivebase_PID_D);
  public PIDController rotPidController = new PIDController(Settings.drivebaseRot_PID_P, Settings.drivebaseRot_PID_I,
      Settings.drivebaseRot_PID_D);

  // path planning vars
  public double trajStartTime;
  public State goalState = new State();
  public double trajElapsedTime;
  public Trajectory.State trajState;
  public Rotation2d goalHeading = new Rotation2d();
  public ChassisSpeeds adjustedSpeeds;
  public PhotonCamera camera = new PhotonCamera(Settings.photonName);
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public double lastPhotonUpdateTime = 0;
  public AHRS gyro;

  // transformation from robot to camera, in two steps, rotation and tranlateion
  Transform3d camToRobotRot = new Transform3d(new Translation3d(), new Rotation3d(0, -0.47, Math.PI));

  Transform3d camToRobotTrans = new Transform3d(new Translation3d(0.305, 0, 0), new Rotation3d());

  //this is estemating camera pose, we will then tranlate this to robot later
  PhotonPoseEstimator photonPoseEstimatorTwo = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, new Transform3d());

  //note detector linearInterp
  public LinearInterp txToNote = new LinearInterp(Settings.tx, Settings.turnDeg);

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
    rotPidController.setIZone(Settings.drivebaseRot_PID_IZ);

    gyro = new AHRS();

    // pathFollowingInit
    goalState = new State();
  }

  // use vision
  public void updateOdometry() {
    if (Settings.usePhoton) {
      var result = camera.getLatestResult();
      Optional<EstimatedRobotPose> pose2Tag = photonPoseEstimatorTwo.update(result);
      if (pose2Tag.isPresent() && goodTagInList(result)) {
        Pose3d fieldToCamera = pose2Tag.get().estimatedPose;
        Pose3d fieldToRobotRot = fieldToCamera.transformBy(camToRobotRot);
        Pose3d fieldToRobot = fieldToRobotRot.transformBy(camToRobotTrans);

        swerveDrive.addVisionMeasurement(fieldToRobot.toPose2d(), pose2Tag.get().timestampSeconds);
        lastPhotonUpdateTime = Timer.getFPGATimestamp();

      }
    }
  }

  // load a path from file
  public void initPath(String pathName, boolean flipToRed) {

    path = PathPlannerPath.fromPathFile(pathName);

    if (flipToRed) {
      path = path.flipPath();
    }

    // geneate trajectory from path
    autoTraj = path.getTrajectory(new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());

    xPosPidController.reset();
    yPosPidController.reset();
    rotPidController.reset();

    goalState = autoTraj.sample(0);
    goalHeading = goalState.targetHolonomicRotation;

    // we need to test, when do we want to force odom, when no vision???
    if (Robot.isSimulation() || thisRobot.autoController.autoRoutine == autoRoutines._XTESTINGACCURACY) {
      setOdomToPathInit();
    }

    trajStartTime = Timer.getFPGATimestamp();

  }

  // apply control to follow each step of path
  public void followPath() {
    // get current sample
    trajElapsedTime = Timer.getFPGATimestamp() - trajStartTime;
    goalState = autoTraj.sample(trajElapsedTime);
    goalHeading = goalState.targetHolonomicRotation;

    // update xy PID controllers
    double xCorrection = xPosPidController.calculate(swerveDrive.getPose().getX(),
        goalState.getTargetHolonomicPose().getX());
    double yCorrection = yPosPidController.calculate(swerveDrive.getPose().getY(),
        goalState.getTargetHolonomicPose().getY());

    setGoalHeadingDeg(goalHeading.getDegrees());

    // add the path velocity with the PID velocity
    Translation2d trajV = new Translation2d(goalState.velocityMps, goalHeading);
    ajustedV = new Translation2d(xCorrection, yCorrection);
    ajustedV = ajustedV.plus(trajV);

    // drive at that velocity
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

  // drive with closed loop heading control while updateing goal heading
  public void driveClosedLoopHeading(Translation2d translation) {

    double rot = rotPidController.calculate(swerveDrive.getOdometryHeading().getRadians(),
        thisRobot.drivebase.goalHeading.getRadians());

    thisRobot.drivebase.swerveDrive.drive(
        translation,
        rot,
        true,
        false);
  }

  public void driveOpenLoopHeading(Translation2d translation, double rV) {

    goalHeading = swerveDrive.getOdometryHeading();
    thisRobot.drivebase.swerveDrive.drive(
        translation,
        rV,
        true,
        false);
  }

  public void attackPoint(Pose2d point) {

    double xP = xPosPidController.calculate(thisRobot.drivebase.swerveDrive.getPose().getX(), point.getX());
    double yP = yPosPidController.calculate(thisRobot.drivebase.swerveDrive.getPose().getY(), point.getY());

    driveClosedLoopHeading(new Translation2d(xP, yP));

  }

  //pubic method to set heading
  public void setGoalHeadingDeg(double deg) {
    goalHeading = new Rotation2d(Math.toRadians(deg));
  }

  //set goal heading to aim at a point on the field, we are always looking away from the goal
  public void aimAtPoint(Translation2d point) {
    Translation2d currentPos = swerveDrive.getPose().getTranslation();
    Translation2d deltaPos = currentPos.minus(point);
    goalHeading = deltaPos.getAngle();
  }

  public boolean inHeadingThold() {
    return Math.abs(swerveDrive.getPose().getRotation().minus(goalHeading).getDegrees()) < Settings.headingThold;
  }

  public boolean inVThold() {
    double xv = swerveDrive.getRobotVelocity().vxMetersPerSecond;
    double yv = swerveDrive.getRobotVelocity().vyMetersPerSecond;
    return Math.sqrt(xv * xv + yv * yv) < Settings.maxShotSpeed;
  }

  public boolean visionIsRecent() {
    return Timer.getFPGATimestamp() - lastPhotonUpdateTime < Settings.stalePhotonTime;
  }

  //used in tele to just update goal heading to face the goals
  public void setGoalHeadingToGoal() {
    if (thisRobot.onRedAlliance) {
      thisRobot.drivebase.aimAtPoint(Settings.redGoalPos);
    } else {
      thisRobot.drivebase.aimAtPoint(Settings.blueGoalPos);
    }
  }

  //determine if tags in list used to update ododm
  public boolean goodTagInList(PhotonPipelineResult res){
    int targets = res.getTargets().size();
    for(int i = 0; i < targets; i++){
      if(Settings.goodTarget.contains(res.getTargets().get(i).getFiducialId())){
        return true;
      }
    }
    return false;
  }

  //used in auto when not moving to point the robot at the goal
  public void aimAtGoal() {
    setGoalHeadingToGoal();
    driveClosedLoopHeading(new Translation2d());
  }

  // drive with closed loop heading control while updateing goal heading
  public void driveRobotCentric(Translation2d translation) {

    double rot = rotPidController.calculate(swerveDrive.getOdometryHeading().getRadians(),
        thisRobot.drivebase.goalHeading.getRadians());

    thisRobot.drivebase.swerveDrive.drive(
        translation,
        rot,
        false,
        false);
  }
  public void aimAtNote(){
    double noteTx = LimelightHelpers.getTX(Settings.llName);
    double ajustGoalHeading = txToNote.interpolateLinearly(noteTx);
    Rotation2d ajustByHeading = new Rotation2d(Math.toRadians(ajustGoalHeading));
    setGoalHeadingDeg(swerveDrive.getOdometryHeading().rotateBy(ajustByHeading).getDegrees());
  }

  public double getDistFromLastPose(){
    Pose2d finalPose = path.getPathPoses().get(path.getPathPoses().size()-1);
    Pose2d currPose = swerveDrive.getPose();
    Transform2d diff = currPose.minus(finalPose);
    return Math.sqrt(diff.getX() * diff.getY() + diff.getY() + diff.getY());
  }

}
