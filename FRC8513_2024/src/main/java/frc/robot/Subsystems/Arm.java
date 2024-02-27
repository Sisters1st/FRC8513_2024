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
    double lastArmGoal = 0;

    public CANSparkMax armMotor1 = new CANSparkMax(Settings.armMotor1CANID, MotorType.kBrushless);
    public CANSparkMax armMotor2 = new CANSparkMax(Settings.armMotor2CANID, MotorType.kBrushless);

    PIDController armPidController = new PIDController(Settings.armPID_P, Settings.armPID_I, Settings.armPID_D);

    public Arm(Robot robotParam) {
        thisRobot = robotParam;

        // set encoder value at boot. allows us to shift zero in future
        armMotor1.getEncoder().setPosition(Settings.armInitRawEncoderValue);

        armMotor1.setSmartCurrentLimit(Settings.armCurrentLimit);
        armMotor2.setSmartCurrentLimit(Settings.armCurrentLimit);

        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);

        armMotor1.getEncoder().setPositionConversionFactor(1);
        armMotor2.getEncoder().setPositionConversionFactor(1);

        armPidController.setIZone(Settings.armPID_IZ);

    }

    // set arm pos taking min max values into account
    public void setArmPosition(double pos) {
        if (pos > Settings.armMaxPos) {
            pos = Settings.armMaxPos;
        }
        if (pos < Settings.armMinPos) {
            pos = Settings.armMinPos;
        }
        armGoalPos = pos;
    }

    public double getArmPosition() {
        return armMotor1.getEncoder().getPosition();
    }

    // update arm motor power
    public void applyArmPower() {

        calculatedArmGoal = armGoalPos;
        // if wer are too far from setpoint, gradually move the setpoint
        if (lastArmGoal + Settings.armMaxV < armGoalPos) {
            calculatedArmGoal = lastArmGoal + Settings.armMaxV;
        }
        if (lastArmGoal - Settings.armMaxV > armGoalPos) {
            calculatedArmGoal = lastArmGoal - Settings.armMaxV;
        }

        double pidPower = armPidController.calculate(getArmPosition(), calculatedArmGoal);
        lastArmGoal = calculatedArmGoal;

        armMotor1.setVoltage((pidPower) * 12);
        armMotor2.setVoltage(-(pidPower) * 12);
    }

    public boolean armWithinThold() {
        return Math.abs(getArmPosition() - calculatedArmGoal) < Settings.armThold;
    }

}
