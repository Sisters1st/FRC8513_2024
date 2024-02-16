package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.Settings;

public class Wrist {

    Robot thisRobot;

    public double wristPos;
    public double wristGoalPos;
    public double calculatedWristGoal;

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

    public void setWristPos(double pos){
        wristGoalPos = pos;
    }

    public double getWristPos(){
        return wristMotor1.getEncoder().getPosition();
    }

    public void applyWristPower(){

        if(calculatedWristGoal < wristGoalPos){
            calculatedWristGoal = calculatedWristGoal + Settings.wristMaxV;
        }
         if(calculatedWristGoal > wristGoalPos){
            calculatedWristGoal = calculatedWristGoal - Settings.wristMaxV;
        }


        double pidPower = wristPidController.calculate(wristMotor1.getEncoder().getPosition(), wristGoalPos);

        wristMotor1.setVoltage((pidPower) * 12);
        wristMotor2.setVoltage(-(pidPower) * 12);
    }


    public boolean wristWithinThold(){
        return Math.abs(wristPos-wristGoalPos) < Settings.wristTHold;
    }
    
}


