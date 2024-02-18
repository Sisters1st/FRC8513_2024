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

        wristMotor1.getEncoder().setPositionConversionFactor(1);
        wristMotor2.getEncoder().setPositionConversionFactor(1);

        wristPidController.setIZone(Settings.wristPID_IZ);

    }

    //set wrist pos with limits
    public void setWristPos(double pos){
        if(pos > Settings.wristMaxPos){
            pos = Settings.wristMaxPos;
        }
        if(pos < Settings.wristMinPos){
            pos = Settings.wristMinPos;
        }
        wristGoalPos = pos;
    }

    public double getWristPos(){
        return wristMotor1.getEncoder().getPosition();
    }

    public void applyWristPower(){

        calculatedWristGoal = wristGoalPos;
        
        //if goal pos is too far away then slowly get there
        if(getWristPos() + Settings.wristMaxV < wristGoalPos){
            calculatedWristGoal = getWristPos() + Settings.wristMaxV;
        }
        if(getWristPos() - Settings.wristMaxV > wristGoalPos){
            calculatedWristGoal = getWristPos() - Settings.wristMaxV;
        }

        double pidPower = wristPidController.calculate(wristMotor1.getEncoder().getPosition(), calculatedWristGoal);

        wristMotor1.setVoltage((pidPower) * 12);
        wristMotor2.setVoltage(-(pidPower) * 12);
    }

    public boolean wristWithinThold(){
        return Math.abs(getWristPos()-wristGoalPos) < Settings.wristTHold;
    }
    
}


