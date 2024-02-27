package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.Settings;

public class Climber {

    Robot thisRobot;

    public CANSparkMax climberMotor1 = new CANSparkMax(Settings.climberMotor1CANID, MotorType.kBrushless);
    public CANSparkMax climberMotor2 = new CANSparkMax(Settings.climberMotor2CANID, MotorType.kBrushless);

    public Climber(Robot robotIn) {
        thisRobot = robotIn;

        climberMotor1.setSmartCurrentLimit(Settings.climberMotorCurrentLimit);
        climberMotor2.setSmartCurrentLimit(Settings.climberMotorCurrentLimit);

    }
}
