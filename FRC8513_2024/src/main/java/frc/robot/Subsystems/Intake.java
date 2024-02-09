package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.Settings;

public class Intake {
    
    Robot thisRobot;
    
    public CANSparkMax leftIntakeMotor = new CANSparkMax(Settings.leftIntakeMotorCANID, MotorType.kBrushless);
    public CANSparkMax rightIntakeMotor = new CANSparkMax(Settings.rightIntakeMotorCANID, MotorType.kBrushless);

    double leftIntakeVoltage = 0;
    double rightIntakeVoltage = 0;

    public Intake(Robot robotIn){
        thisRobot = robotIn;

        leftIntakeMotor.setSmartCurrentLimit(Settings.intakeMotorCurrnetLimit);
        rightIntakeMotor.setSmartCurrentLimit(Settings.intakeMotorCurrnetLimit);

        leftIntakeMotor.setIdleMode(IdleMode.kBrake);
        rightIntakeMotor.setIdleMode(IdleMode.kBrake);

    }

    public void setIntakeVoltage(double voltage){
        leftIntakeVoltage = voltage;
        rightIntakeVoltage = voltage;
    }

    public void applyIntakeVoltage(){
        leftIntakeMotor.setVoltage(leftIntakeVoltage);
        rightIntakeMotor.setVoltage(rightIntakeVoltage);
    }


}