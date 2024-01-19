// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {

  public final TalonFX m_driveMotor;
  public final CANSparkMax m_turningMotor;

  public final AnalogEncoder m_turnEncoder;

  public final PIDController m_drivePIDController = new PIDController(Settings.drivePID_P, Settings.drivePID_I, Settings.drivePID_D);

  public SwerveModuleState goalSwerveState = new SwerveModuleState();

  public final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          Settings.turnPID_P,
          Settings.turnPID_I,
          Settings.turnPID_D,
          new TrapezoidProfile.Constraints(
              Settings.moduleMaxAngularVelocity, Settings.moduleMaxAngularAcceleration));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(Settings.driveKS, Settings.driveKV);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(Settings.turnKS,Settings.driveKV);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID CANID for the drive motor.
   * @param turningMotorID CANID for the turning motor.
   * @param turningEncoderPort Analog input for the turn encoder
   */
  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turningEncoderPort) {

    m_driveMotor = new TalonFX(driveMotorID);
    m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    m_turnEncoder = new AnalogEncoder(turningEncoderPort);
    m_turnEncoder.setDistancePerRotation(Settings.turnEncoderVoltagToDegreesRatio);

    // Reminder to setup drive encoder ticks per revolution

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getVelocity().getValue(), new Rotation2d(m_turnEncoder.getAbsolutePosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getPosition().getValue(), new Rotation2d(m_turnEncoder.getAbsolutePosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turnEncoder.getAbsolutePosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
    goalSwerveState = state;
    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getVelocity().getValue(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turnEncoder.getAbsolutePosition(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
