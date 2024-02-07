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

    public static int opperatingArmJoystickPort = 1;
    public static int climberJoystickPort = 2;

     //Button settings
    public static int shootInSpeakerButton = 5;
    public static int ampPrepButton = 4;
    public static int climberPrepButton = 7;
    public static int climbButton = 7;
    public static int intakeButton = 1;
    public static int drivingStateReturnButton = 2;
    public static int runFeederButton= 8;

    //pdh settings
    public static int pdhCANID = 1;

    //heading settings
    public static boolean headingCorrection = true;
    public static double hc_P = 0.1;
    public static double hc_I = 0;
    public static double hc_D = 0;

    //intake settings
    public static int leftIntakeMotorCANID = 61;
    public static int rightIntakeMotorCANID = 62;

    public static int intakeMotorCurrnetLimit = 20;
    public static double intakingVoltage = 8;

    //Arm settings
    public static int armMotor1CANID = 14;
    public static int armMotor2CANID = 15;
    public static double armInitRawEncoderValue = 0;
    public static double armEncoderToDegreeRatio = 90000;

    public static double armReductin = 0;
    public static double armFF = 0;

    public static int arm1CurrentLimit = 30;
    public static int arm2CurentLimit = 30;

    public static double armPID_P = 0;
    public static double armPID_I = 0;
    public static double armPID_D = 0;

    public static double intakingArmPos = 0;
    public static double trapArmPos = 0;
    public static double ampArmPos = 0;

    //Wrist settings
    public static int wristMotor1CANID = 64;
    public static int wristMotor2CANID = 63;
    public static double wristInitRawEncoderValue = 0;
    public static double wristEncoderToDegreeRatio = 0;

    public static double wristReduction = 0;
    public static double wristFF = 0;

    public static int wrist1CurrentLimit = 10;
    public static int wrist2CurrentLimit = 10;

    public static double wristPID_P = 0;
    public static double wristPID_I = 0;
    public static double wristPID_D = 0;

    public static double intakingWristPos = 0;
    public static double trapWristPos = 0;
    public static double ampWristPos = 0;

    //feeder settings
    public static int feederMotorCANID = 58;

    //Shooter settings
    public static int leftShooterCANID = 51;
    public static int rightShooterCANID = 52;
    public static int feederCANID = 53;

    public static double shooter_P = 0;
    public static double shooter_I = 0;
    public static double shooter_D = 0;

    public static double shooterThresholdValue = 10;

    public static int shooter1CurrentLimit = 10;
    public static int shooter2CurrentLimit = 10;
    public static int feederCurrentLimit = 10;

    public static double feederScoreTrapVoltage = 8;
    public static double feederScoreAmpVoltage = 8;
    public static double feederIntakeVoltage = -8;

    //climber settings

    public static int climberMotor1CANID = 55;
    public static int climberMotor2CANID = 56;
    public static int climberMotorCurrentLimit = 40;
    public static double climberVoltage = 8.0;
        

}
