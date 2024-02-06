package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
    
    Robot thisRobot;
    

    CANSparkMax leftIntake = new CANSparkMax(Settings.leftIntakeMotorCANID, MotorType.kBrushless);
    CANSparkMax rightIntake = new CANSparkMax(Settings.rightIntakeMotorCANID, MotorType.kBrushless);

}
