package frc.robot;

public class Settings {

    public static int frontLeftDriveMotorCANID = 0;
    public static int frontLeftTurnMotorCANID = 0;
    public static int frontLeftTurnEncoderPort = 0;

    public static int frontRightDriveMotorCANID = 0;
    public static int frontRightTurnMotorCANID = 0;
    public static int frontRightTurnEncoderPort = 0;

    public static int backLeftDriveMotorCANID = 0;
    public static int backLeftTurnMotorCANID = 0;
    public static int backLeftTurnEncoderPort = 0;

    public static int backRightDriveMotorCANID = 0;
    public static int backRightTurnMotorCANID = 0;
    public static int backTurnrightEncoderPort = 0;

    public static double wheelRadius = 0.0508;
    public static double turnEncoderVoltagToDegreesRatio = 360.0/5.0;
    public static double driveEncoderToMetersRatio = -1;
    public static double moduleMaxAngularVelocity = 0;
    public static double kModuleMaxAngularAcceleration = 2 * Math.PI;

    public static double drivePID_P = 0.0;
    public static double drivePID_I = 0.0;
    public static double drivePID_D = 0.0;

    public static double driveKV = 0;
    public static double driveKS = 0;


    public static double turnPID_P = 0.0;
    public static double turnPID_I = 0.0;
    public static double turnPID_D = 0.0;

    public static double turnKV = 0;
    public static double turnKS = 0;


    
}
