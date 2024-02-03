package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class Shooter {

    Robot thisRobot;

    CANSparkFlex leftShooter = new CANSparkFlex(Settings.leftShooterCANID, MotorType.kBrushless);
    CANSparkFlex rightShooter = new CANSparkFlex(Settings.rightShooterCANID, MotorType.kBrushless);

    PIDController leftShooterPIDController = new PIDController(Settings.shooter_P, Settings.shooter_I, Settings.shooter_D);
    PIDController rightShooterPIDController = new PIDController(Settings.shooter_P, Settings.shooter_I, Settings.shooter_D);
    
    double leftShooterGoalSpeed;
    double rightShooterGoalSpeed;

    public Shooter(Robot robotIn){
        thisRobot = robotIn;
    }

    public void setShooterSpeeds(double lss, double rss){
        leftShooterGoalSpeed = lss;
        rightShooterGoalSpeed = rss;
    }

    public void updateMotorPower(){
        if(leftShooterGoalSpeed == 0){
            leftShooter.setVoltage(0);
        }else{
            double leftShooterPower = leftShooterPIDController.calculate(leftShooter.getEncoder().getVelocity(), leftShooterGoalSpeed);
            leftShooter.setVoltage(leftShooterPower);
        }
        if(rightShooterGoalSpeed == 0){
            rightShooter.setVoltage(0);
        }else{
            double rightShooterPower = rightShooterPIDController.calculate(rightShooter.getEncoder().getVelocity(), rightShooterGoalSpeed);
            rightShooter.setVoltage(rightShooterPower);
        }
    }

}
