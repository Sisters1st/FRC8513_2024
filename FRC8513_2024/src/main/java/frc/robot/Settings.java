package frc.robot;

public class Settings {

    //module settings
    public static int frontLeftDriveMotorCANID = 51;
    public static int frontLeftTurnMotorCANID = 52;
    public static int frontLeftTurnEncoderPort = 0;
    public static double frontLeftDriveXOffset = 0;
    public static double frontLeftDriveYOffset = 0;

    public static int frontRightDriveMotorCANID = 53;
    public static int frontRightTurnMotorCANID = 54;
    public static int frontRightTurnEncoderPort = 1;
    public static double frontRightDriveXOffset = 0;
    public static double frontRightDriveYOffset = 0;

    public static int backLeftDriveMotorCANID = 55;
    public static int backLeftTurnMotorCANID = 56;
    public static int backLeftTurnEncoderPort = 2;
    public static double backLeftDriveXOffset = 0;
    public static double backLeftDriveYOffset = 0;

    public static int backRightDriveMotorCANID = 57;
    public static int backRightTurnMotorCANID = 58;
    public static int backRightTurnEncoderPort = 3;
    public static double backRightDriveXOffset = 0;
    public static double backRightDriveYOffset = 0;

    public static double moduleMaxAngularVelocity = 0;
    public static double moduleMaxAngularAcceleration = 2 * Math.PI;

    public static double wheelRadius = 0.0508;
    public static double turnEncoderVoltagToDegreesRatio = 360.0/5.0;
    public static double driveEncoderToMetersRatio = -1;

    public static double drivePID_P = 2;
    public static double drivePID_I = 3;
    public static double drivePID_D = 9;

    public static double driveKV = 0;
    public static double driveKS = 0;

    public static double turnPID_P = 0.0;
    public static double turnPID_I = 0.0;
    public static double turnPID_D = 0.0;

    public static double turnKV = 0;
    public static double turnKS = 0;

    //drivebase settings
    public static double maxDBSpeed = 0.0;
    public static double maxDBAngularSpeed = Math.PI;

    //dashbord and logging settings
    public static int dashboardNumber = 3;

    //vision settings
    public static boolean useLimelight = false;
    public static String limelightName = "limelight";

    //joystick settings
    public static int driverJoystickPort = 0;
    public static int driver_fieldCentricButton = 20;

    //teleop default settings
    public static boolean teleopFieldCentricDefault = true;

    //pdh settings
    public static int pdhCANID = 1;
}
