package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class Wrist {

    Robot thisRobot;

    double rawWristPosition;
    double wristAngleToArm;
    double wristAngleToGround;
    double wristGoalAngleToGround;

    CANSparkMax wristMotor1 = new CANSparkMax(Settings.wristMotor1CANID, MotorType.kBrushless);
    CANSparkMax wristMotor2 = new CANSparkMax(Settings.wristMotor2CANID, MotorType.kBrushless);

    PIDController wristPidController = new PIDController(Settings.wristPID_P, Settings.wristPID_I, Settings.wristPID_D);

    public Wrist(Robot robotParam){
        thisRobot = robotParam;

        wristMotor2.follow(wristMotor1);
        wristMotor2.setInverted(true);

        wristMotor1.getEncoder().setPosition(Settings.wristInitRawEncoderValue);

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

        wristAngleToGround = wristAngleToArm + ( thisRobot.arm).getArmPosition();

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
    
}


