package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Settings {

    // drivebase and yagsl settings
    public static double drivebaseMaxVelocity = 17.1; //fps, gets converted to mps later
    public static double maxShotSpeed = 0.25;
    public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.MACHINE;
    public static double drivebase_PID_P = 4; //x and y path following PID
    public static double drivebase_PID_I = 0.25;
    public static double drivebase_PID_D = 0.2;

    // heading settings
    public static boolean attackPointEnable = false;
    public static double headingThold = 4;
    public static double rotErrorToRPSRatio = 3.0; 
    public static double minRotSpeed = 0.15;

    public static double drivebaseRot_PID_P = 6;
    public static double drivebaseRot_PID_I = 0.1;
    public static double drivebaseRot_PID_D = 0.85;
    public static double drivebaseRot_PID_IZ = 5;

    // photon settings (at) //10.85.13.27:5800
    public static boolean usePhoton = true;
    public static String photonName = "LL3";
    public static double stalePhotonTime = 5; // ip for photon: http://10.85.13.26:5800/#/dashboard
    public static ArrayList<Integer> goodTarget = new ArrayList<>(Arrays.asList(3,4,7,8));
    public static double minTargetArea = 0.35;

    //ll settings (not tracking) 10.85.31.28:5801
    public static String llName = "limelight";
        // tx to turn degrees
    public static double[] tx = {      -30, -10, 0, 10, 30};
    public static double[] turnDeg = { -20, -10, 0, 10, 20};


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
    public static int aimAtNoteAxis = 2;

    // Button pannel settings
    public static int buttonPannelPort = 1;

    public static int drivingStateReturnButton = 2;
    public static int intakeButton = 3;
    public static int shootInSpeakerButton = 5;
    public static int ampPrepButton = 8;
    public static int climberPrepButton = 9;
    public static int intakeOutButton = 1;
    public static int climbUpButton = 11;
    public static int climbDownButton = 10;
    public static int runFeederInButton = 7;
    public static int runFeederOutButton = 4;
    public static int coinButton = 6;

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

    public static int intakeMotorCurrentLimit = 35;
    public static double intakingVoltage = 12;
    public static int intakeSensorPort = 4;
    public static double intakeSensorThold = 750;

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
    public static double climbArmPos = -36;
    public static double chainGrabArmPos = -52;
    public static double ampArmPos = -35;
    public static double shootingArmPos = -24;

    public static double armThold = 1.0;
    public static double armMaxV = 1.0;
    public static double armMaxPos = 0;
    public static double armMinPos = -65;

    // Wrist settings

    // tuned for new notes at mt olive
    public static double[] shotDistances = { 0, 1.28, 1.58, 1.8, 2.02, 2.4, 5};
    public static double[] shotWristPos = { 0.5, 0.5, -1.83, -2.49, -3.68, -4.8, -4.8};

    public static int wristMotor1CANID = 60;
    public static int wristMotor2CANID = 61;
    public static double wristInitRawEncoderValue = 0;

    public static int wristCurrentLimit = 15;

    public static double wristPID_P = 0.05;
    public static double wristPID_I = 0.1;
    public static double wristPID_D = 0;
    public static double wristPID_IZ = 3;

    public static double intakingWristPos = 7;
    public static double climbWristpos = -35;
    public static double chainGrabWristpos = -5;
    public static double ampWristPos = -2;
    public static double shootingSubwofferWristPos = shotWristPos[0];
    public static double shootingSubwofferOtherWristPos = shotWristPos[0];

    public static double wristMotorDiff = 0.025;
    public static double wristTHold = 1;
    public static double wristMaxV = 3.0;
    public static double wristMaxPos = 10;
    public static double wristMinPos = -45;

    // Shooter settings
    public static int leftShooterCANID = 41;
    public static int rightShooterCANID = 42;
    public static double maxShotDistance = 2.5;

    public static double shooter_P = 0.003;
    public static double shooter_I = 0.0005;
    public static double shooter_D = 0;
    public static double shooter_FF = 0.0017;
    public static double shooter_FF_const = 0;

    public static double shooterThresholdValue = 100;
    public static double shotTime = 0.65;

    public static int shooter1CurrentLimit = 35;
    public static int shooter2CurrentLimit = 35;

    public static int basicShooterSpeed = 3000;
    public static double leftRightShooterSpeedOffset = 1;
    public static double matchShooterOverideDelta = 0.20;
    public static double matchShooterOveride = -1;
    public static double waitGoodShotTime = 0.15;
    public static double shootingDist = 1.0;

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
    public static double shimmyInVoltage = 8;

    public static double shimmyOutPos = -3;
    public static double shimmyInPos = 2;
    public static int shimmyCount = 2;
    public static double feedInDist = 1;

    // climber settings

    public static int climberMotor1CANID = 55;
    public static int climberMotor2CANID = 56;
    public static int climberMotorCurrentLimit = 55;
    public static double climberDistance = 310;

    // field settings

    public static Translation2d blueGoalPos = new Translation2d(0.15, 5.54); //7.4 inches from front of tag. Tag is -1.5 x
    public static Translation2d redGoalPos = new Translation2d(16.39, 5.54);
    public static Pose2d blueAmp = new Pose2d(1.82, 7.65, new Rotation2d());
    public static Pose2d redAmp = new Pose2d(14.73, 7.65, new Rotation2d(Math.PI));

}
