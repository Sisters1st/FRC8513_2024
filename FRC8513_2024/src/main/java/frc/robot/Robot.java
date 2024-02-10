package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
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

  //logic
  public Dashboard dashboard = new Dashboard(this, Settings.dashboardNumber);
  public TeleopController teleopController = new TeleopController(this);
  public AutoController autoController = new AutoController(this);
  public StateMachine stateMachine = new StateMachine(this);

  //subsystems
  public PowerDistribution pdh = new PowerDistribution(Settings.pdhCANID, ModuleType.kRev);
  public Drivebase drivebase = new Drivebase(this);
  public Arm arm = new Arm(this);
  public Wrist wrist = new Wrist(this);
  public Shooter shooter = new Shooter(this);
  public Climber climber = new Climber(this);
  public Intake intake = new Intake(this);
  
  public boolean lastUserButton = false;
  public boolean onRedAlliance = false;

  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {
    drivebase.updateOdometry();
    dashboard.updateDashboard();  

    if(RobotController.getUserButton() != lastUserButton){
      lastUserButton = RobotController.getUserButton();
      //user button is buggy maybe get another way of doing this
      if(RobotController.getUserButton()){
        arm.armMotor1.setIdleMode(IdleMode.kCoast);
        arm.armMotor2.setIdleMode(IdleMode.kCoast);

        wrist.wristMotor1.setIdleMode(IdleMode.kCoast);
        wrist.wristMotor1.setIdleMode(IdleMode.kCoast);
      } else {
        arm.armMotor1.setIdleMode(IdleMode.kBrake);
        arm.armMotor2.setIdleMode(IdleMode.kBrake);

        wrist.wristMotor1.setIdleMode(IdleMode.kBrake);
        wrist.wristMotor1.setIdleMode(IdleMode.kBrake);

      }
    }
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

  public void updateAllSubsystemMotorPower(){
    arm.applyArmPower();
    wrist.applyWristPower();
    shooter.applyShooterPower();
    intake.applyIntakeVoltage();
  }
}
