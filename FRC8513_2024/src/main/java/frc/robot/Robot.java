package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Logic.AutoController;
import frc.robot.Logic.Dashboard;
import frc.robot.Logic.Leds;
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

  // logic
  public Dashboard dashboard = new Dashboard(this);
  public TeleopController teleopController = new TeleopController(this);
  public AutoController autoController = new AutoController(this);
  public StateMachine stateMachine = new StateMachine(this);
  public Orchestra m_orchestra = new Orchestra();
  public LinearInterp linearInterp;

  // subsystems
  public PowerDistribution pdh = new PowerDistribution(Settings.pdhCANID, ModuleType.kRev);
  public Drivebase drivebase = new Drivebase(this);
  public Arm arm = new Arm(this);
  public Wrist wrist = new Wrist(this);
  public Shooter shooter = new Shooter(this);
  public Climber climber = new Climber(this);
  public Intake intake = new Intake(this);
  public Leds m_led = new Leds(this);

  // robot wide vars
  public boolean onRedAlliance = false;
  public double wristOveride = Settings.matchShooterOveride;
  public boolean playMusic = false;
  public boolean dontShoot = false;

  @Override
  public void robotInit() {
    if (Robot.isSimulation()) {
      Settings.usePhoton = false;
    }
    double[] shotDistances = Settings.shotDistances;
    double[] wristPos = Settings.shotWristPos;
    linearInterp = new LinearInterp(shotDistances, wristPos);

    var audioConfig = new AudioConfigs();
    audioConfig.withAllowMusicDurDisable(true); 

    ((TalonFX)drivebase.swerveDrive.getModules()[0].getDriveMotor().getMotor()).getConfigurator().apply(audioConfig);
    ((TalonFX)drivebase.swerveDrive.getModules()[1].getDriveMotor().getMotor()).getConfigurator().apply(audioConfig);
    ((TalonFX)drivebase.swerveDrive.getModules()[2].getDriveMotor().getMotor()).getConfigurator().apply(audioConfig);
    ((TalonFX)drivebase.swerveDrive.getModules()[3].getDriveMotor().getMotor()).getConfigurator().apply(audioConfig);


    m_orchestra.addInstrument((TalonFX)drivebase.swerveDrive.getModules()[0].getDriveMotor().getMotor());
    m_orchestra.addInstrument((TalonFX)drivebase.swerveDrive.getModules()[1].getDriveMotor().getMotor());
    m_orchestra.addInstrument((TalonFX)drivebase.swerveDrive.getModules()[2].getDriveMotor().getMotor());
    m_orchestra.addInstrument((TalonFX)drivebase.swerveDrive.getModules()[3].getDriveMotor().getMotor());
    
    m_orchestra.loadMusic("diamonds.chrp");
    
    
  }

  @Override
  public void robotPeriodic() {
    drivebase.updateOdometry();
    dashboard.updateDashboard();
    m_led.updateLeds();
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
    m_orchestra.stop();
    m_led.changeLedColor(0, 0, 0);
    teleopController.init();
  }

  @Override
  public void teleopPeriodic() {
    teleopController.periodic();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    
    autoController.autoDisabeled();

    if(teleopController.buttonPannel.getRawButtonPressed(Settings.coinButton)){
      playMusic = !playMusic;
    }

    if(playMusic && !m_orchestra.isPlaying()){
      m_orchestra.play();
      
    } 

    if(!playMusic && m_orchestra.isPlaying()){
      m_orchestra.stop();
    } 

    if(playMusic){
      m_led.updateRainbow();
    } else {
      m_led.changeLedColor(0, 0, 0);

    }

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  // after all vars are upated, actually apply the motor power
  public void updateAllSubsystemMotorPower() {
    arm.applyArmPower();
    wrist.applyWristPower();
    shooter.applyShooterPower();
    intake.applyIntakeVoltage();
  }

  // check DS for alliance color
  public void updateAlliance() {
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
