package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.Orchestra;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Logic.AutoController;
import frc.robot.Logic.Dashboard;
import frc.robot.Logic.LinearInterp;
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
  public Dashboard dashboard = new Dashboard(this);
  public TeleopController teleopController = new TeleopController(this);
  public AutoController autoController = new AutoController(this);
  public StateMachine stateMachine = new StateMachine(this);
  public LinearInterp linearInterp;

  //subsystems
  public PowerDistribution pdh = new PowerDistribution(Settings.pdhCANID, ModuleType.kRev);
  public Drivebase drivebase = new Drivebase(this);
  public Arm arm = new Arm(this);
  public Wrist wrist = new Wrist(this);
  public Shooter shooter = new Shooter(this);
  public Climber climber = new Climber(this);
  public Intake intake = new Intake(this);
  
  //robot wide vars
  public boolean lastUserButton = false;
  public boolean onRedAlliance = false;
  public double wristOveride = Settings.matchShooterOveride;
  Orchestra m_orchestra = new Orchestra();
  TalonFX instrument;

  @Override
  public void robotInit() {
    if(Robot.isSimulation()){
      Settings.usePhoton = false;
    }
    double[] shotDistances = Settings.shotDistances;
    double[] wristPos = Settings.shotWristPos;
    linearInterp = new LinearInterp(shotDistances, wristPos);

  }

  @Override
  public void robotPeriodic() {
    drivebase.updateOdometry();
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
  public void disabledPeriodic() {

    //if user button pressed set arm to break mode
    if(RobotController.getUserButton() != lastUserButton){
      lastUserButton = RobotController.getUserButton();
      
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
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  //after all vars are upated, actually apply the motor power
  public void updateAllSubsystemMotorPower(){
    arm.applyArmPower();
    wrist.applyWristPower();
    shooter.applyShooterPower();
    intake.applyIntakeVoltage();
  }

  //check DS for alliance color
  public void updateAlliance(){
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            onRedAlliance = true;
        }
        if (ally.get() == Alliance.Blue) {
            onRedAlliance = false;
        }
    }
  }
}
