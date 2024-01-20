package frc.robot;

public class Settings {

    //module settings
    public static double distanceFromCenterToModuleInM = 0.276;

    public static int frontLeftDriveMotorCANID = 6;
    public static int frontLeftTurnMotorCANID = 5;
    public static int frontLeftTurnEncoderPort = 0;
    public static double frontLeftDriveXOffset = distanceFromCenterToModuleInM;
    public static double frontLeftDriveYOffset = distanceFromCenterToModuleInM;
    public static double frontLeftTurnOffset = 270;

    public static int frontRightDriveMotorCANID = 7;
    public static int frontRightTurnMotorCANID = 8;
    public static int frontRightTurnEncoderPort = 1;
    public static double frontRightDriveXOffset = distanceFromCenterToModuleInM;
    public static double frontRightDriveYOffset = -distanceFromCenterToModuleInM;
    public static double frontRightTurnOffset = 138;

    public static int backLeftDriveMotorCANID = 9;
    public static int backLeftTurnMotorCANID = 2;
    public static int backLeftTurnEncoderPort = 2;
    public static double backLeftDriveXOffset = -distanceFromCenterToModuleInM;
    public static double backLeftDriveYOffset = distanceFromCenterToModuleInM;
    public static double backLeftTurnOffset = 125;

    public static int backRightDriveMotorCANID = 3;
    public static int backRightTurnMotorCANID = 4;
    public static int backRightTurnEncoderPort = 3;
    public static double backRightDriveXOffset = -distanceFromCenterToModuleInM;
    public static double backRightDriveYOffset = -distanceFromCenterToModuleInM;
    public static double backRightTurnOffset = 198;

    public static int maxTurnCurrent = 20;
    public static int maxDriveCurrent = 40;

    public static double moduleMaxAngularVelocity = -1;// 6 * Math.PI;
    public static double moduleMaxAngularAcceleration = -1; // 8 * Math.PI;

    public static double wheelRadius = 0.0508;
    public static double turnEncoderVoltagToDegreesRatio = 360.0;
    public static double driveEncoderToMetersRatio = 6.75 / (2*Math.PI*wheelRadius);

    public static double drivePID_P = 1;
    public static double drivePID_I = 0;
    public static double drivePID_D = 0;

    public static double driveKV = 0;
    public static double driveKS = 0;

    public static double turnPID_P = 1;
    public static double turnPID_I = 0.0;
    public static double turnPID_D = 0;

    public static double turnKV = 0;
    public static double turnKS = 0;

    //drivebase settings
    public static double maxDBSpeed = 4.7;
    public static double maxDBAngularSpeed = 6 * Math.PI;

    //dashbord and logging settings
    public static int dashboardNumber = 3;

    //vision settings
    public static boolean useLimelight = false;
    public static String limelightName = "limelight";

    //joystick settings
    public static int driverJoystickPort = 0;
    public static int driver_fieldCentricButton = 5;

    //teleop default settings
    public static boolean teleopFieldCentricDefault = true;

    //pdh settings
    public static int pdhCANID = 1;
}
