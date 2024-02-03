package frc.robot;
    
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class Arm {

    Robot thisRobot;

    double rawArmPosition;
    double armAngle;
    double armGoalAngle;

    CANSparkMax armMotor1 = new CANSparkMax(Settings.armMotor1CANID, MotorType.kBrushless);
    CANSparkMax armMotor2 = new CANSparkMax(Settings.armMotor2CANID, MotorType.kBrushless);

    PIDController armPidController = new PIDController(Settings.armPID_P, Settings.armPID_I, Settings.armPID_D);

    public Arm(Robot robotParam){
        thisRobot = robotParam;

        armMotor2.follow(armMotor1);
        armMotor2.setInverted(true);

        armMotor1.getEncoder().setPosition(Settings.armInitRawEncoderValue);

    }

    public void setArmPosition(double degrees){
        armGoalAngle = degrees;
    }

    public double getArmPosition(){
        return armAngle;
    }

    public double updateArmAngle(){
        rawArmPosition = armMotor1.getEncoder().getPosition();
        armAngle = rawArmPosition * Settings.armEncoderToDegreeRatio;
        return armAngle;
    }

    public void updateArmMotorPower(){
        double pidPower = armPidController.calculate(getArmPosition(), armGoalAngle);
        double ffPower = calculateFFTerm();

        armMotor1.setVoltage((pidPower + ffPower) * 12);
    }

    public double calculateFFTerm(){
        double cosOfAng = Math.cos(getArmPosition());
        double ffPower = cosOfAng * Settings.armFF;

        return ffPower;
    }
    
}


