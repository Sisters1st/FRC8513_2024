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

    //pdh settings
    public static int pdhCANID = 1;

    //heading settings
    public static boolean headingCorrection = true;
    public static double hc_P = 0.1;
    public static double hc_I = 0;
    public static double hc_D = 0;

    

    public static double joyBand = 0.1;
}
