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
    public double wristLastGoalPos = 0;

    public CANSparkMax wristMotor1 = new CANSparkMax(Settings.wristMotor1CANID, MotorType.kBrushless);
    public CANSparkMax wristMotor2 = new CANSparkMax(Settings.wristMotor2CANID, MotorType.kBrushless);

    PIDController wristRPidController = new PIDController(Settings.wristPID_P, Settings.wristPID_I, Settings.wristPID_D);
    PIDController wristLPidController = new PIDController(Settings.wristPID_P, Settings.wristPID_I, Settings.wristPID_D);


    public Wrist(Robot robotParam) {
        thisRobot = robotParam;

        wristMotor1.getEncoder().setPosition(Settings.wristInitRawEncoderValue);

        wristMotor1.setSmartCurrentLimit(Settings.wristCurrentLimit);
        wristMotor2.setSmartCurrentLimit(Settings.wristCurrentLimit);

        wristMotor1.setIdleMode(IdleMode.kBrake);
        wristMotor2.setIdleMode(IdleMode.kBrake);

        wristMotor1.getEncoder().setPositionConversionFactor(1);
        wristMotor2.getEncoder().setPositionConversionFactor(1);

        wristRPidController.setIZone(Settings.wristPID_IZ);
        wristLPidController.setIZone(Settings.wristPID_IZ);

    }

    // set wrist pos with limits
    public void setWristPos(double pos) {
        if (pos > Settings.wristMaxPos) {
            pos = Settings.wristMaxPos;
        }
        if (pos < Settings.wristMinPos) {
            pos = Settings.wristMinPos;
        }
        wristGoalPos = pos;
    }

    public double getWristPos() {
        return wristMotor1.getEncoder().getPosition();
    }

    public void applyWristPower() {

        calculatedWristGoal = wristGoalPos;

        // if goal pos is too far away then slowly get there
        if (wristLastGoalPos + Settings.wristMaxV < wristGoalPos) {
            calculatedWristGoal = wristLastGoalPos + Settings.wristMaxV;
        }
        if (wristLastGoalPos - Settings.wristMaxV > wristGoalPos) {
            calculatedWristGoal = wristLastGoalPos - Settings.wristMaxV;
        }

        double pidLeftPower = wristLPidController.calculate(wristMotor1.getEncoder().getPosition(), calculatedWristGoal + 1);
        double pidRightPower = wristRPidController.calculate(wristMotor1.getEncoder().getPosition(), calculatedWristGoal);
        wristLastGoalPos = calculatedWristGoal;

        wristMotor1.setVoltage((pidLeftPower) * 12);
        wristMotor2.setVoltage(-(pidRightPower) * 12);
    }

    public boolean wristWithinThold() {
        return Math.abs(getWristPos() - wristGoalPos) < Settings.wristTHold;
    }

}
