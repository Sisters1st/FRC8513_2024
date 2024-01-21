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
}
