package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Robot extends TimedRobot {

  public Drivebase drivebase = new Drivebase(this);
  public Dashboard dashboard = new Dashboard(this, Settings.dashboardNumber);
  public TeleopController teleopController = new TeleopController(this);

  public PowerDistribution pdh = new PowerDistribution(Settings.pdhCANID, ModuleType.kRev);

  @Override
  public void robotInit() {

    Settings.turnPID_P = Preferences.getDouble("turnP",Settings.turnPID_P);
    Settings.turnPID_I = Preferences.getDouble("turnI",Settings.turnPID_I);
    Settings.turnPID_D = Preferences.getDouble("turnD",Settings.turnPID_D);

    
    Settings.drivePID_P = Preferences.getDouble("driveP",Settings.drivePID_P);
    Settings.drivePID_I = Preferences.getDouble("driveI",Settings.drivePID_I);
    Settings.drivePID_D = Preferences.getDouble("driveD",Settings.drivePID_D);

  }

  @Override
  public void robotPeriodic() {
    dashboard.updateDashboard();
    drivebase.updateOdometry();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    drivebase.m_frontRight.m_driveMotor.set(1);
  }

  @Override
  public void teleopInit() {
    teleopController.init();
  }

  @Override
  public void teleopPeriodic() {
    teleopController.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
