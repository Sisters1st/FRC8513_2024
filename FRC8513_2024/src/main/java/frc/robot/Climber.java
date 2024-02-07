package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber {

    Robot thisRobot;

    CANSparkMax climberMotor1 = new CANSparkMax(Settings.climberMotor1CANID, MotorType.kBrushless);
    CANSparkMax climberMotor2 = new CANSparkMax(Settings.climberMotor2CANID, MotorType.kBrushless);

    public Climber(Robot robotIn){
        thisRobot = robotIn;

        climberMotor1.setSmartCurrentLimit(Settings.climberMotorCurrentLimit);
        climberMotor2.setSmartCurrentLimit(Settings.climberMotorCurrentLimit);
        
    }


    
}
