package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Logic.AutoController;
import frc.robot.Logic.Dashboard;
import frc.robot.Logic.StateMachine;
import frc.robot.Logic.TeleopController;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Wrist;

public class Robot extends TimedRobot {

  public Dashboard dashboard = new Dashboard(this, Settings.dashboardNumber);
  public TeleopController teleopController = new TeleopController(this);
  public AutoController autoController = new AutoController(this);
  public StateMachine stateMachine = new StateMachine(this);

  public PowerDistribution pdh = new PowerDistribution(Settings.pdhCANID, ModuleType.kRev);
  public Drivebase drivebase = new Drivebase(this);
  public Arm arm = new Arm(this);
  public Wrist wrist = new Wrist(this);
  public Shooter shooter = new Shooter(this);
  public Climber climber = new Climber(this);
  public Intake intake = new Intake(this);

  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {
    drivebase.updateOdometry();
    arm.updateArmAngle();
    wrist.updateWristPositions();
    dashboard.updateDashboard();
  }

  @Override
  public void autonomousInit() {
    autoController.autoInit();
  }

  @Override
  public void autonomousPeriodic() {
    autoController.autoPeriodic();
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
  public void simulationInit() {
    drivebase.simulateDrivebaseInit();
  }

  @Override
  public void simulationPeriodic() {
    drivebase.simulateDrivebase();
  }
}
