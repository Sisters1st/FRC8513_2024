package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.Settings;

public class Intake {
    
    Robot thisRobot;
    
    public CANSparkMax leftIntake = new CANSparkMax(Settings.leftIntakeMotorCANID, MotorType.kBrushless);
    public CANSparkMax rightIntake = new CANSparkMax(Settings.rightIntakeMotorCANID, MotorType.kBrushless);

    public Intake(Robot robotIn){
        thisRobot = robotIn;

        leftIntake.setSmartCurrentLimit(Settings.intakeMotorCurrnetLimit);
        rightIntake.setSmartCurrentLimit(Settings.intakeMotorCurrnetLimit);

        leftIntake.setIdleMode(IdleMode.kBrake);
        rightIntake.setIdleMode(IdleMode.kBrake);

    }

    public void setIntakeVoltage(double voltage){
        leftIntake.setVoltage(-voltage);
        rightIntake.setVoltage(-voltage);
    }

}