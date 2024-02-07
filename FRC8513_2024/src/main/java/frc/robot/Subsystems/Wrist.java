package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.Settings;

public class Wrist {

    Robot thisRobot;

    public double rawWristPosition;
    public double wristAngleToArm;
    public double wristAngleToGround;
    public double wristGoalAngleToGround;

    public CANSparkMax wristMotor1 = new CANSparkMax(Settings.wristMotor1CANID, MotorType.kBrushless);
    public CANSparkMax wristMotor2 = new CANSparkMax(Settings.wristMotor2CANID, MotorType.kBrushless);

    PIDController wristPidController = new PIDController(Settings.wristPID_P, Settings.wristPID_I, Settings.wristPID_D);

    public Wrist(Robot robotParam){
        thisRobot = robotParam;

        wristMotor1.getEncoder().setPosition(Settings.wristInitRawEncoderValue);

        wristMotor1.setSmartCurrentLimit(Settings.wrist1CurrentLimit);
        wristMotor2.setSmartCurrentLimit(Settings.arm2CurentLimit);

        wristMotor1.setIdleMode(IdleMode.kBrake);
        wristMotor2.setIdleMode(IdleMode.kBrake);
    }

    public void setWristPositionToGround(double degrees){
        wristGoalAngleToGround = degrees;
    }

    public double getWristToGroundPosition(){
        return wristAngleToGround;
    }

    public double calculateThetaAW(){
        double thetaA = thisRobot.arm.getArmPosition();
        double thetaW = wristAngleToGround;
        
        double thetaAW = (180-thetaA) + thetaW;

        return thetaAW;
    }

    public void updateWristPositions(){
        rawWristPosition = wristMotor1.getEncoder().getPosition();
        wristAngleToArm = rawWristPosition * Settings.wristEncoderToDegreeRatio;
        wristAngleToGround = calculateThetaAW();

    }

    public void updateWristMotorPower(){
        double pidPower = wristPidController.calculate(getWristToGroundPosition(), wristGoalAngleToGround);
        double ffPower = calculateFFTerm();

        wristMotor1.setVoltage((pidPower + ffPower) * 12);
    }

    public double calculateFFTerm(){
        double cosOfAng = Math.cos(getWristToGroundPosition());
        double ffPower = cosOfAng * Settings.wristFF;

        return ffPower;
    }

    public boolean wristWithinThold(){
        return Math.abs(wristAngleToGround-wristGoalAngleToGround) < Settings.wristTHold;
    }
    
}


