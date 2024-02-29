package frc.robot.Logic;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;

public class StateMachine {

    Robot thisRobot;
    public robotStates robotState = robotStates.DRIVING;
    double lastStateChangeTime = 0; // track last time state changed
    double shotStartedTime = -1; // track when a shot was initiated
    boolean comittedToShot = false; // once we start a shot, we need to finish it
    int shimmyCount = Settings.shimmyCount; // time for shimmy
    double shimmyStartDist = 0;
    boolean shimmyIn = true;
    boolean forceShooterOn = false;

    // each of the subsystem vars to keep track of
    double feederV = 0;
    double armPos = 0;
    double wristPos = 0;
    double intakeVoltage = 0;
    double ss = 0;

    public StateMachine(Robot robotIn) {
        thisRobot = robotIn;
    }

    public void updateRobotState() {

        switch (robotState) {
            // driving around, intake off
            case DRIVING:
                armPos = Settings.intakingArmPos;
                wristPos = Settings.intakingWristPos;
                ss = feederV = intakeVoltage = 0;

                break;

            // run the intake, feeder in
            case INTAKING:
                armPos = Settings.intakingArmPos;
                wristPos = Settings.intakingWristPos;
                intakeVoltage = Settings.intakingVoltage;
                feederV = Settings.feederIntakeVoltage;
                ss = 0;

                // if sensor gets tripped, go back to driving
                if (thisRobot.shooter.intakeSensorSeesNote()) {
                    robotState = robotStates.DRIVING;
                    lastStateChangeTime = Timer.getFPGATimestamp();
                    shimmyCount = 0;
                }
                break;

            // runs once when we initilize a shot
            case SPEEDING_UP_SHOOTER_SPEAKER:

                // if vision, get dist, if no vision, assume sw shot
                armPos = Settings.shootingArmPos;
                if (thisRobot.drivebase.visionIsRecent()) {
                    wristPos = getWristAngFromDist(thisRobot.shooter.getDistFromGoal());
                } else {
                    wristPos = Settings.shootingSubwofferWristPos;
                }

                feederV = intakeVoltage = 0;
                ss = Settings.basicShooterSpeed;
                shotStartedTime = -1;
                comittedToShot = false;

                // move on to the shot
                robotState = robotStates.SHOOTING;

                break;

            case SHOOTING:
                armPos = Settings.shootingArmPos;

                // if vision, get dist, if no vision, assume sw shot
                if (thisRobot.drivebase.visionIsRecent()) {
                    wristPos = getWristAngFromDist(thisRobot.shooter.getDistFromGoal());
                    //wristPos = thisRobot.wrist.wristGoalPos + 0.1 * thisRobot.teleopController.manualControlJoystick.getRawAxis(Settings.manualControlWristAxis);
                } else {
                    wristPos = Settings.shootingSubwofferWristPos + thisRobot.wristOveride;
                }
                feederV = intakeVoltage = 0;
                ss = Settings.basicShooterSpeed;
                if (robotInAllTHolds() || comittedToShot) {
                    feederV = Settings.feederIntakeVoltage;

                    if (shotStartedTime == -1) {
                        shotStartedTime = Timer.getFPGATimestamp();
                        comittedToShot = true;
                    }

                    if (Timer.getFPGATimestamp() - shotStartedTime > Settings.shotTime) {
                        robotState = robotStates.DRIVING;
                    }

                } else {
                    feederV = 0;
                }

                break;
            case CLIMBING:
                if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.climberPrepButton)){
                    armPos = Settings.chainGrabArmPos;
                    wristPos = Settings.chainGrabWristpos;

                } else {
                    armPos = Settings.climbArmPos;
                    wristPos = Settings.climbWristpos;
                }

                ss = feederV = intakeVoltage = 0;

                manualClimberControl();
                break;

            case SCORE_AMP:

                armPos = Settings.ampArmPos;
                wristPos = Settings.ampWristPos;
                ss = feederV = intakeVoltage = 0;

                break;

            default:
                break;
        }
        checkAllButtonsForStateChanges();
        if (robotState != robotStates.INTAKING) {
            freeFeederControl();
            freeIntakeControl();
        }

        thisRobot.arm.setArmPosition(armPos);
        thisRobot.wrist.setWristPos(wristPos);

        // may have to add logic which brings note back to sensor after shimmy
        if (shimmyCount == 0) {
            if (thisRobot.shooter.intakeSensorSeesNote()) {
                thisRobot.shooter.setFeederVoltage(-Settings.shimmyInVoltage);
            } else {
                shimmyCount = 1;
                shimmyStartDist = thisRobot.shooter.getFeederPos();
                shimmyIn = true;
            }
        } else {
            if (shimmyCount < Settings.shimmyCount) {
                if (shimmyIn) {
                    thisRobot.shooter.setFeederVoltage(Settings.shimmyInVoltage);
                    if (thisRobot.shooter.intakeSensorSeesNote()) {
                        shimmyIn = false;
                        shimmyStartDist = thisRobot.shooter.getFeederPos();
                        shimmyCount++;
                    }
                } else {
                    thisRobot.shooter.setFeederVoltage(-Settings.shimmyInVoltage);
                    if (shimmyStartDist - thisRobot.shooter.getFeederPos() > Settings.shimmyDist) {
                        shimmyIn = true;
                        shimmyStartDist = thisRobot.shooter.getFeederPos();
                    }
                }
            } else {
                thisRobot.shooter.setFeederVoltage(feederV);
            }
        }
        thisRobot.intake.setIntakeVoltage(intakeVoltage);
        if (forceShooterOn) {
            thisRobot.shooter.setShooterSpeeds(Settings.basicShooterSpeed);
        } else {
            thisRobot.shooter.setShooterSpeeds(ss);
        }

    }

    // manual control of the feeder
    public void freeFeederControl() {
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.runFeederInButton)) {
            feederV = Settings.feederIntakeVoltage;
        }
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.runFeederOutButton)) {
            feederV = Settings.feederScoreAmpVoltage;
        }
    }

    // manual spit out of intake
    public void freeIntakeControl() {
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.intakeOutButton)) {
            intakeVoltage = -Settings.intakingVoltage;
        }
    }

    // from any state, check if we need to go into another state
    public void checkAllButtonsForStateChanges() {
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.shootInSpeakerButton)) {
            robotState = robotStates.SPEEDING_UP_SHOOTER_SPEAKER;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.intakeButton)) {
            robotState = robotStates.INTAKING;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.climberPrepButton)) {
            robotState = robotStates.CLIMBING;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.ampPrepButton)) {
            robotState = robotStates.SCORE_AMP;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.drivingStateReturnButton)) {
            robotState = robotStates.DRIVING;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
    }

    public void manualClimberControl() {
        if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbDownButton) == false &&
            thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbUpButton) == false){
                
            double lc = thisRobot.teleopController.manualControlJoystick.getRawAxis(Settings.manualControlArmAxis);
            double rc = thisRobot.teleopController.manualControlJoystick.getRawAxis(Settings.manualControlWristAxis);
            //cm 2 is left climber
            //+ roll is to the right
            //+ roll means subtract from left clmber
            //- roll means add that negative roll to riht climber
            //lc + power is climb up
            //rc - power is climb up
            thisRobot.climber.climberMotor1.set(rc);
            thisRobot.climber.climberMotor2.set(-lc);
        } else {
            
            double roll = thisRobot.drivebase.gyro.getRoll();
            double climbPowerDelta = roll * 0.1;
            if(climbPowerDelta > 1){
                climbPowerDelta = 1;
            }
            if(climbPowerDelta < -1){
                climbPowerDelta = -1;
            }
            if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbDownButton)){
                thisRobot.climber.climberMotor1.set(-1);
                thisRobot.climber.climberMotor2.set(1);
            } else {
                thisRobot.climber.climberMotor1.set(1 + climbPowerDelta);
                thisRobot.climber.climberMotor2.set(-1 + climbPowerDelta);

            }


        }

    }

    // generated from cubic line of best fit. will need to get retuned
    public double getWristAngFromDist(double dist) {
        double wristVal = thisRobot.linearInterp.interpolateLinearly(dist) + thisRobot.wristOveride;
        return wristVal;
    }

    // checks all subsustems in tholds. if vision is out of date, dont check range
    public boolean robotInAllTHolds() {
        if (thisRobot.drivebase.visionIsRecent()) {
            return thisRobot.shooter.shootersWithinThold()
                    && thisRobot.arm.armWithinThold()
                    && thisRobot.wrist.wristWithinThold()
                    && thisRobot.drivebase.inHeadingThold()
                    && thisRobot.drivebase.inVThold()
                    && thisRobot.shooter.shotWithinRange()
                    && !thisRobot.teleopController.buttonPannel.getRawButton(Settings.shootInSpeakerButton);
        }
        return thisRobot.shooter.shootersWithinThold()
                && thisRobot.arm.armWithinThold()
                && thisRobot.wrist.wristWithinThold()
                && thisRobot.drivebase.inHeadingThold()
                && thisRobot.drivebase.inVThold()
                && !thisRobot.teleopController.buttonPannel.getRawButton(Settings.shootInSpeakerButton);
    }

    public void forceRobotState(robotStates inState) {
        lastStateChangeTime = Timer.getFPGATimestamp();
        robotState = inState;
    }

    public enum robotStates {
        DRIVING,
        INTAKING,
        SHOOTING,
        SPEEDING_UP_SHOOTER_SPEAKER,
        SCORE_AMP,
        CLIMBING
    }
}
