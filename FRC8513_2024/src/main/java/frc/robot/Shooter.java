package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class Shooter {

    Robot thisRobot;

    CANSparkFlex leftShooter = new CANSparkFlex(Settings.leftShooterCANID, MotorType.kBrushless);
    CANSparkFlex rightShooter = new CANSparkFlex(Settings.rightShooterCANID, MotorType.kBrushless);

    CANSparkMax feederMotor = new CANSparkMax(Settings.feederCANID, MotorType.kBrushless);

    PIDController leftShooterPIDController = new PIDController(Settings.shooter_P, Settings.shooter_I, Settings.shooter_D);
    PIDController rightShooterPIDController = new PIDController(Settings.shooter_P, Settings.shooter_I, Settings.shooter_D);
    
    double leftShooterGoalSpeed;
    double rightShooterGoalSpeed;

    public Shooter(Robot robotIn){
        thisRobot = robotIn;

        leftShooter.setSmartCurrentLimit(Settings.shooter1CurrentLimit);
        rightShooter.setSmartCurrentLimit(Settings.shooter2CurrentLimit);
        feederMotor.setSmartCurrentLimit(Settings.feederCurrentLimit);

        rightShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setIdleMode(IdleMode.kCoast);
        feederMotor.setIdleMode(IdleMode.kBrake);
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

    public boolean leftShooterInThreshold(){
        if (Math.abs(leftShooter.getEncoder().getVelocity() - leftShooterGoalSpeed) < Settings.shooterThresholdValue);
            return true;
        }

    public boolean rightShooterInThreshold(){
        if (Math.abs(rightShooter.getEncoder().getVelocity() - rightShooterGoalSpeed) < Settings.shooterThresholdValue);
            return true;
        }
    }


