package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Settings {


    //drivebase and yagsl settings
    public static double drivebaseMaxVelocity = 15.1;
    public static double maxShotSpeed = 0.2;
    public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.MACHINE;


    //vision settings
    public static boolean usePhoton = true;
    public static String photonName = "LLCamera";
    public static double stalePhotonTime = 10.0; //ip for photon: http://10.85.13.26:5800/#/dashboard


    //driver joystick settings
    public static int driverJoystickPort = 0;
    public static boolean manualTestingControl = false;

    public static double joystickDeadband = 0.01;
    public static double rotJoyRate = 10;
        
        //driver axis and buttons
    public static int leftRightAxis = 0;
    public static int forwardBackwardsAxis = 1;
    public static int rotAxis = 4; //2 at home, 4 on xbox
        //driver buttons
    public static int snapToAmpButton = 1;
    public static int snapToSpeakerButton = 5;
    public static int aimAtSpeakerButton = 6;
    public static int resetFieldCentricButton = 8;
    

    //Button pannel settings
    public static int buttonPannelPort = 1;

    public static int drivingStateReturnButton = 1;
    public static int intakeButton = 2;
    public static int shootInSpeakerButton = 3;
    public static int climberPrepButton = 4;
    public static int ampPrepButton = 5;
    public static int intakeOutButton = 6;
    public static int climbUpButton = 7;
    public static int climbDownButton = 8;
    public static int runFeederInButton= 9;
    public static int runFeederOutButton = 10;


    //manual overide joystick settings
    public static int manualControlPort = 2;

    public static int manualControlArmAxis = 1;
    public static int manualControlWristAxis = 5;

    public static int manualResetZeroButton = 1;
    public static int enableManualControlButton1 = 5;
    public static int enableManualControlButton2 = 6;


    //pdh settings
    public static int pdhCANID = 1;

    
    //heading settings
    public static boolean headingCorrection = true;
    public static double hc_P = 0.15;
    public static double hc_I = 0;
    public static double hc_D = 0.01;
    public static double headingThold = 2;


    //intake settings
    public static int leftIntakeMotorCANID = 51;
    public static int rightIntakeMotorCANID = 52;

    public static int intakeMotorCurrnetLimit = 30;
    public static double intakingVoltage = 12;


    //Arm settings
    public static int armMotor1CANID = 14;
    public static int armMotor2CANID = 15;
    public static double armInitRawEncoderValue = 0;

    public static int armCurrentLimit = 30;

    public static double armPID_P = 0.1;
    public static double armPID_I = 0.3;
    public static double armPID_D = 0;
    public static double armPID_IZ = 3;

    public static double intakingArmPos = -4;
    public static double trapArmPos = -56;
    public static double ampArmPos = -41;
    public static double shootingArmPos = -19;

    public static double armThold = 1;
    public static double armMaxV = 1.33;
    public static double armMaxPos = 0;
    public static double armMinPos = -55;


    //Wrist settings
    public static int wristMotor1CANID = 60;
    public static int wristMotor2CANID = 61;
    public static double wristInitRawEncoderValue = 0;

    public static int wristCurrentLimit = 15;

    public static double wristPID_P = 0.02;
    public static double wristPID_I = 0.05;
    public static double wristPID_D = 0;
    public static double wristPID_IZ = 1;

    public static double intakingWristPos = 3.5;
    public static double trapWristPos = 1;
    public static double ampWristPos = -1;
    public static double shootingSubwofferWristPos = -7;

    public static double wristTHold = 1;
    public static double wristMaxV = 2;
    public static double wristMaxPos = 13;
    public static double wristMinPos = -15;


    //Shooter settings
    public static int leftShooterCANID = 41;
    public static int rightShooterCANID = 42;
    public static double maxShotDistance = 3.0;

    public static double shooter_P = 0.002;
    public static double shooter_I = 0;
    public static double shooter_D = 0;
    public static double shooter_FF = 0.0021;
    public static double shooter_FF_const = 1.42;
    
    public static double shooterThresholdValue = 50;
    public static double shotTime = 0.8;

    public static int shooter1CurrentLimit = 30;
    public static int shooter2CurrentLimit = 30;
    
    public static int basicShooterSpeed = 5250;
    public static double leftRightShooterSpeedOffset = 0.9;


    //feeder settings
    public static int feederCANID = 58;
    public static int feederCurrentLimit = 25;

    public static boolean useFeederSensor = true;
    public static int feederSensorPort = 6; //on mxp
    public static double feederNoteThold = 425;
    
    public static double feederScoreTrapVoltage = -8;
    public static double feederScoreAmpVoltage = -8;
    public static double feederIntakeVoltage = 9;
    public static double shimmyInVoltage = 6;

    public static double shimmyDist = 0.8;
    public static int shimmyCount = 6;
    

    //climber settings

    public static int climberMotor1CANID = 55;
    public static int climberMotor2CANID = 56;
    public static int climberMotorCurrentLimit = 40;
    public static double climberVoltage = 8.0;

    //field settings
    
    public static Translation2d blueGoalPos = new Translation2d(0.0381,5.54736);
    public static Translation2d redGoalPos = new Translation2d(16.579342,5.54736);
        

}
