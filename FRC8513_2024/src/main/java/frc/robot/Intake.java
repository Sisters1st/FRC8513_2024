package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
    
    Robot thisRobot;
    
    CANSparkMax leftIntake = new CANSparkMax(Settings.leftIntakeMotorCANID, MotorType.kBrushless);
    CANSparkMax rightIntake = new CANSparkMax(Settings.rightIntakeMotorCANID, MotorType.kBrushless);

    public Intake(Robot robotIn){
        thisRobot = robotIn;

        leftIntake.setSmartCurrentLimit(Settings.intakeMotorCurrnetLimit);
        rightIntake.setSmartCurrentLimit(Settings.intakeMotorCurrnetLimit);

        leftIntake.setIdleMode(IdleMode.kBrake);
        rightIntake.setIdleMode(IdleMode.kBrake);

    }

    public void setIntakeVoltage(double voltage){
        leftIntake.setVoltage(voltage);
        rightIntake.setVoltage(-voltage);
    }

}
