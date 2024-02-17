package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Robot;
import frc.robot.Settings;

public class Shooter {

    Robot thisRobot;

    public CANSparkFlex leftShooter = new CANSparkFlex(Settings.leftShooterCANID, MotorType.kBrushless);
    public CANSparkFlex rightShooter = new CANSparkFlex(Settings.rightShooterCANID, MotorType.kBrushless);

    public CANSparkMax feederMotor = new CANSparkMax(Settings.feederCANID, MotorType.kBrushless);

    PIDController leftShooterPIDController = new PIDController(Settings.shooter_P, Settings.shooter_I, Settings.shooter_D);
    PIDController rightShooterPIDController = new PIDController(Settings.shooter_P, Settings.shooter_I, Settings.shooter_D);
    
    public double leftShooterGoalSpeed = 0;
    public double rightShooterGoalSpeed = 0;
    double feederVoltage = 0;
    public AnalogInput feederSensorInput = new AnalogInput(Settings.feederSensorPort);

    public Shooter(Robot robotIn){
        thisRobot = robotIn;

        leftShooter.setSmartCurrentLimit(Settings.shooter1CurrentLimit);
        rightShooter.setSmartCurrentLimit(Settings.shooter2CurrentLimit);
        feederMotor.setSmartCurrentLimit(Settings.feederCurrentLimit);

        rightShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setIdleMode(IdleMode.kCoast);
        feederMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setShooterSpeeds(double lss, double rss, double feeder){
        leftShooterGoalSpeed = -lss;
        rightShooterGoalSpeed = rss;
        feederVoltage = feeder;
        rightShooterPIDController.reset();
        leftShooterPIDController.reset();

    }

    public void applyShooterPower(){
        double leftFF = leftShooterGoalSpeed * Settings.shooter_FF;
        double rightFF = rightShooterGoalSpeed * Settings.shooter_FF;

        if(leftShooterGoalSpeed == 0){
            leftShooter.setVoltage(0);
        }else{
            double leftShooterPower = leftShooterPIDController.calculate(leftShooter.getEncoder().getVelocity(), leftShooterGoalSpeed);
            leftShooter.setVoltage(12);//setVoltage(leftShooterPower + leftFF);
        }
        if(rightShooterGoalSpeed == 0){
            rightShooter.setVoltage(0);
        }else{
            double rightShooterPower = rightShooterPIDController.calculate(rightShooter.getEncoder().getVelocity(), rightShooterGoalSpeed);
            rightShooter.setVoltage(12); //setVoltage(rightShooterPower + rightFF);
        }

        feederMotor.setVoltage(feederVoltage);
    }

    public boolean leftShooterInThreshold(){
        return Math.abs(leftShooter.getEncoder().getVelocity() - leftShooterGoalSpeed) < Settings.shooterThresholdValue;
    }


    public boolean rightShooterInThreshold(){
        return Math.abs(rightShooter.getEncoder().getVelocity() - rightShooterGoalSpeed) < Settings.shooterThresholdValue;
    }

    public boolean intakeSensorSeesNote(){
        if(Settings.useFeederSensor){
            return feederSensorInput.getValue() > Settings.feederNoteThold;
        } else {
            return false;
        }
    }

}


