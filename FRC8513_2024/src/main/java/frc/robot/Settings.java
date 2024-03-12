package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Settings {

    // drivebase and yagsl settings
    public static double drivebaseMaxVelocity = 17.1; //fps, gets converted to mps later
    public static double maxShotSpeed = 1;
    public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.MACHINE;
    public static double drivebase_PID_P = 12; //x and y path following PID
    public static double drivebase_PID_I = 0;
    public static double drivebase_PID_D = 0.25;

    // heading settings
    public static boolean attackPointEnable = false;
    public static double headingThold = 4;

    public static double drivebaseRot_PID_P = 5;
    public static double drivebaseRot_PID_I = 0;
    public static double drivebaseRot_PID_D = 0.5;
    public static double drivebaseRot_PID_IZ = 3;

    // vision settings
    public static boolean usePhoton = true;
    public static String photonName = "LLCamera";
    public static double stalePhotonTime = 15; // ip for photon: http://10.85.13.26:5800/#/dashboard

    // driver joystick settings
    public static int driverJoystickPort = 0;
    public static boolean manualTestingControl = false;

    public static double joystickDeadband = 0.01;
    public static double rotJoyRate = 10;

    // driver axis and buttons
    public static int leftRightAxis = 0;
    public static int forwardBackwardsAxis = 1;
    public static int rotAxis = 4; // 2 at home, 4 on xbox
    // driver buttons
    public static int snapToAmpButton = 1;
    public static int aimAtSpeakerButton = 6;
    public static int resetFieldCentricButton = 8;

    // Button pannel settings
    public static int buttonPannelPort = 1;

    public static int drivingStateReturnButton = 1;
    public static int intakeButton = 2;
    public static int shootInSpeakerButton = 3;
    public static int ampPrepButton = 5;
    public static int climberPrepButton = 4;
    public static int intakeOutButton = 6;
    public static int climbUpButton = 7;
    public static int climbDownButton = 8;
    public static int runFeederInButton = 9;
    public static int runFeederOutButton = 10;
    public static int coinButton = 11;

    // manual overide joystick settings
    public static int manualControlPort = 2;

    public static int manualControlArmAxis = 1; //also left climber
    public static int manualControlWristAxis = 5; //also right climber

    public static int manualResetZeroButton = 1; //arm wrist
    public static int enableManualControlButton1 = 5;
    public static int enableManualControlButton2 = 6;

    // pdh settings
    public static int pdhCANID = 1;

    // intake settings
    public static int leftIntakeMotorCANID = 51;
    public static int rightIntakeMotorCANID = 52;

    public static int intakeMotorCurrentLimit = 30;
    public static double intakingVoltage = 12;

    // Arm settings
    public static int armMotor1CANID = 14;
    public static int armMotor2CANID = 15;
    public static double armInitRawEncoderValue = 0;

    public static int armCurrentLimit = 30;

    public static double armPID_P = 0.1;
    public static double armPID_I = 0.3;
    public static double armPID_D = 0;
    public static double armPID_IZ = 3;

    public static double intakingArmPos = -6;
    public static double climbArmPos = -38;
    public static double chainGrabArmPos = -48;
    public static double ampArmPos = -35;
    public static double shootingArmPos = -24;

    public static double armThold = 1;
    public static double armMaxV = 1;
    public static double armMaxPos = 0;
    public static double armMinPos = -65;

    // Wrist settings

    // tuned for new notes at mt olive
    public static double[] shotDistances = { 1.00, 1.27, 1.48, 1.67, 1.82, 2.0, 8.0 };
    public static double[] shotWristPos = { -1.35, -1.35, -3.07, -3.95, -4.02, -4.57, -4.57 };

    public static int wristMotor1CANID = 60;
    public static int wristMotor2CANID = 61;
    public static double wristInitRawEncoderValue = 0;

    public static int wristCurrentLimit = 15;

    public static double wristPID_P = 0.05;
    public static double wristPID_I = 0.1;
    public static double wristPID_D = 0;
    public static double wristPID_IZ = 3;

    public static double intakingWristPos = 7;
    public static double climbWristpos = -38;
    public static double chainGrabWristpos = -10;
    public static double ampWristPos = -2;
    public static double shootingSubwofferWristPos = shotWristPos[0];
    public static double shootingSubwofferOtherWristPos = shotWristPos[0];

    public static double wristMotorDiff = 0.05;
    public static double wristTHold = 1;
    public static double wristMaxV = 3;
    public static double wristMaxPos = 10;
    public static double wristMinPos = -45;

    // Shooter settings
    public static int leftShooterCANID = 41;
    public static int rightShooterCANID = 42;
    public static double maxShotDistance = 2.5;

    public static double shooter_P = 0.002;
    public static double shooter_I = 0.0001;
    public static double shooter_D = 0;
    public static double shooter_FF = 0.0021;
    public static double shooter_FF_const = 1.42;

    public static double shooterThresholdValue = 100;
    public static double shotTime = 0.65;

    public static int shooter1CurrentLimit = 35;
    public static int shooter2CurrentLimit = 35;

    public static int basicShooterSpeed = 5500;
    public static double leftRightShooterSpeedOffset = 1;
    public static double matchShooterOveride = -0.6;
    public static double matchShooterOverideDelta = 0.1;

    // feeder settings
    public static int feederCANID = 58;
    public static int feederCurrentLimit = 25;

    public static boolean useFeederSensor = true;
    public static int feederSensorPort = 6; // on mxp
    public static double feederNoteThold = 325;

    public static double feederScoreTrapVoltage = -8;
    public static double feederScoreAmpVoltage = -8;
    public static double feederIntakeVoltage = 9;
    
    public static double feederShooterVoltage = 12;
    public static double shimmyInVoltage = 6;

    public static double shimmyDist = 0.5;
    public static int shimmyCount = 4;

    // climber settings

    public static int climberMotor1CANID = 55;
    public static int climberMotor2CANID = 56;
    public static int climberMotorCurrentLimit = 55;
    public static double climberVoltage = 8.0;
    public static double climberDistance = 200;

    // field settings

    public static Translation2d blueGoalPos = new Translation2d(0.076, 5.547868);
    public static Translation2d redGoalPos = new Translation2d(16.465042, 5.547868);
    public static Pose2d blueAmp = new Pose2d(1.82, 7.7, new Rotation2d());
    public static Pose2d redAmp = new Pose2d(14.73, 7.7, new Rotation2d());

}
