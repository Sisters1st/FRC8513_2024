package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.Settings;

public class Intake {

    Robot thisRobot;

    public CANSparkMax leftIntakeMotor = new CANSparkMax(Settings.leftIntakeMotorCANID, MotorType.kBrushless);
    public CANSparkMax rightIntakeMotor = new CANSparkMax(Settings.rightIntakeMotorCANID, MotorType.kBrushless);
    public AnalogInput intakeSensor = new AnalogInput(Settings.intakeSensorPort);

    double leftIntakeVoltage = 0;
    double rightIntakeVoltage = 0;

    public Intake(Robot robotIn) {
        thisRobot = robotIn;

        leftIntakeMotor.setSmartCurrentLimit(Settings.intakeMotorCurrentLimit);
        rightIntakeMotor.setSmartCurrentLimit(Settings.intakeMotorCurrentLimit);

        leftIntakeMotor.setIdleMode(IdleMode.kCoast);
        rightIntakeMotor.setIdleMode(IdleMode.kCoast);

        intakeSensor.setAverageBits(3);

    }

    public void setIntakeVoltage(double voltage) {
        leftIntakeVoltage = voltage;
        rightIntakeVoltage = voltage;
    }

    public void applyIntakeVoltage() {
        leftIntakeMotor.setVoltage(leftIntakeVoltage);
        rightIntakeMotor.setVoltage(rightIntakeVoltage);
    }

    public boolean intakeSensorSeesNote(){
        return intakeSensor.getAverageValue() > Settings.intakeSensorThold;
    }

}