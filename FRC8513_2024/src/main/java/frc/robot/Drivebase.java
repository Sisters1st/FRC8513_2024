package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Drivebase {

  public static final double kMaxSpeed = Settings.maxDBSpeed;
  public static final double kMaxAngularSpeed = Settings.maxDBAngularSpeed;

  private final Translation2d m_frontLeftLocation = new Translation2d(Settings.frontLeftDriveXOffset, Settings.frontLeftDriveYOffset);
  private final Translation2d m_frontRightLocation = new Translation2d(Settings.frontRightDriveXOffset, Settings.frontRightDriveYOffset);
  private final Translation2d m_backLeftLocation = new Translation2d(Settings.backLeftDriveXOffset, Settings.backLeftDriveYOffset);
  private final Translation2d m_backRightLocation = new Translation2d(Settings.backRightDriveXOffset, Settings.backRightDriveYOffset);

  public final SwerveModule m_frontLeft = new SwerveModule(Settings.frontLeftDriveMotorCANID, Settings.frontLeftTurnMotorCANID, Settings.frontLeftTurnEncoderPort);
  public final SwerveModule m_frontRight = new SwerveModule(Settings.frontRightDriveMotorCANID, Settings.frontRightTurnMotorCANID, Settings.frontRightTurnEncoderPort);
  public final SwerveModule m_backLeft = new SwerveModule(Settings.backLeftDriveMotorCANID, Settings.backLeftTurnMotorCANID, Settings.backLeftTurnEncoderPort);
  public final SwerveModule m_backRight = new SwerveModule(Settings.backRightDriveMotorCANID, Settings.backRightTurnMotorCANID, Settings.backRightTurnEncoderPort);

  public final AHRS m_gyro = new AHRS();

  public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public final SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          }, new Pose2d());

  public Drivebase() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
    
    if(Settings.useLimelight){
      m_odometry.addVisionMeasurement(LimelightHelpers.getBotPose2d(Settings.limelightName),LimelightHelpers.getLatency_Pipeline(Settings.limelightName));
    }
    
  }
}


