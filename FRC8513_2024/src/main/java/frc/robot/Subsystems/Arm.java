package frc.robot.Subsystems;
    
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.Settings;

public class Arm {

    Robot thisRobot;

    public double armPos;
    public double armGoalPos;
    public double calculatedArmGoal = 0;

    public CANSparkMax armMotor1 = new CANSparkMax(Settings.armMotor1CANID, MotorType.kBrushless);
    public CANSparkMax armMotor2 = new CANSparkMax(Settings.armMotor2CANID, MotorType.kBrushless);

    PIDController armPidController = new PIDController(Settings.armPID_P, Settings.armPID_I, Settings.armPID_D);

    public Arm(Robot robotParam){
        thisRobot = robotParam;

        armMotor1.getEncoder().setPosition(Settings.armInitRawEncoderValue);

        armMotor1.setSmartCurrentLimit(Settings.arm1CurrentLimit);
        armMotor2.setSmartCurrentLimit(Settings.arm2CurentLimit);

        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);

        armMotor1.getEncoder().setPositionConversionFactor(1);
        armMotor2.getEncoder().setPositionConversionFactor(1);

    }

    public void setArmPosition(double pos){
        armGoalPos = pos;
    }

    public double getArmPosition(){
        return armMotor1.getEncoder().getPosition();
    }

    public void updateArmMotorPower(){

        if(calculatedArmGoal < armGoalPos){
            calculatedArmGoal = calculatedArmGoal + Settings.armMaxV;
        }
         if(calculatedArmGoal > armGoalPos){
            calculatedArmGoal = calculatedArmGoal - Settings.armMaxV;
        }

        double pidPower = armPidController.calculate(armMotor1.getEncoder().getPosition(), calculatedArmGoal);
        double ffPower = calculateFFTerm();

        armMotor1.setVoltage((pidPower + ffPower) * 12);
        armMotor2.setVoltage(-(pidPower + ffPower) * 12);
    }

    public double calculateFFTerm(){
        double cosOfAng = Math.cos(getArmPosition());
        double ffPower = cosOfAng * Settings.armFF;

        return ffPower;
    }

    public boolean armWithinThold(){
        return Math.abs(armPos-armGoalPos) < Settings.armThold;
    }
    
}


