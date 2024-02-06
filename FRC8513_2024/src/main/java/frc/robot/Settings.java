package frc.robot;

import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Settings {

    //drivebase settings
    public static double drivebaseMaxVelocity = 15.1;

    //dashbord and logging settings
    public static int dashboardNumber = 3;
    public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;

    //vision settings
    public static boolean useLimelight = false;
    public static String limelightName = "limelight";

    //joystick settings
    public static int driverJoystickPort = 0;
    public static int driver_fieldCentricButton = 5;
    public static double joyBand = 0.1;

    public static int opperatingArmJoystickPort = 0;

    public static int climberJoystickPort = 0;

     //Button settings
    public static int shootInSpeakerButton = 5;
    public static int shootInAMPWarmUpButton = 3;
    public static int shootInAMPButton = 4;
    public static int climberButton = 7;
    public static int intakeButton = 1;
    public static int drivingStateReturnButton = 2;
    public static int shootingInTrap = 8;

    //pdh settings
    public static int pdhCANID = 1;

    //heading settings
    public static boolean headingCorrection = true;
    public static double hc_P = 0.1;
    public static double hc_I = 0;
    public static double hc_D = 0;

    //Arm settings
    public static int armMotor1CANID = 0;
    public static int armMotor2CANID = 0;
    public static double armInitRawEncoderValue = 0;
    public static double armEncoderToDegreeRatio = 90000;

    public static double armReductin = 0;
    public static double armFF = 0;

    public static int arm1CurrentLimit = 10;
    public static int arm2CurentLimit = 10;

    public static double armPID_P = 0;
    public static double armPID_I = 0;
    public static double armPID_D = 0;

    //Wrist settings
    public static int wristMotor1CANID = 0;
    public static int wristMotor2CANID = 0;
    public static double wristInitRawEncoderValue = 0;
    public static double wristEncoderToDegreeRatio = 0;

    public static double wristReduction = 0;
    public static double wristFF = 0;

    public static int wrist1CurrentLimit = 10;
    public static int wrist2CurrentLimit = 10;

    public static double wristPID_P = 0;
    public static double wristPID_I = 0;
    public static double wristPID_D = 0;

    //Shooter settings
    public static int leftShooterCANID = 51;
    public static int rightShooterCANID = 52;

    public static double shooter_P = 0;
    public static double shooter_I = 0;
    public static double shooter_D = 0;

    public static double thresholdValue = 10;

    public static int shooter1CurrentLimit = 10;
    public static int shooter2CurrentLimit = 10;

}
