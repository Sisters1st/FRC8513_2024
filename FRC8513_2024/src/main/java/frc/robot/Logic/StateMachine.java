package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
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
    int climbCounter = 0;
    boolean needToFeedIn = false;
    double feedInStartDist = -1;
    double firstTimeGood = 0;

    public StateMachine(Robot robotIn) {
        thisRobot = robotIn;
    }

    public void updateRobotState() {

        switch (robotState) {
            // driving around, intake off
            case DRIVING:
                climbCounter = 0;
                armPos = Settings.intakingArmPos;
                wristPos = Settings.intakingWristPos;
                ss = feederV = intakeVoltage = 0;
                break;

            // run the intake, feeder in
            case INTAKING:
                climbCounter = 0;
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
                climbCounter = 0;
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
                firstTimeGood = -1;

                break;

            case SHOOTING:
                armPos = Settings.shootingArmPos;

                // if vision, get dist, if no vision, assume sw shot
                if (thisRobot.drivebase.visionIsRecent()) {
                    wristPos = getWristAngFromDist(thisRobot.shooter.getDistFromGoal());
                    //wristPos = thisRobot.wrist.wristGoalPos + 0.1 *
                    //thisRobot.teleopController.manualControlJoystick.getRawAxis(Settings.manualControlWristAxis);
                } else {
                    wristPos = Settings.shootingSubwofferWristPos + thisRobot.wristOveride;
                }
                feederV = intakeVoltage = 0;
                ss = Settings.basicShooterSpeed;
                if ((robotInAllTHolds() || comittedToShot) && firstTimeGood == -1) {
                    comittedToShot = true;
                    firstTimeGood = Timer.getFPGATimestamp();
                }
                if(Timer.getFPGATimestamp() - firstTimeGood > Settings.waitGoodShotTime && comittedToShot){
                    feederV = Settings.feederShooterVoltage;

                    if (shotStartedTime == -1) {
                        shotStartedTime = Timer.getFPGATimestamp();
                    }

                    if (Timer.getFPGATimestamp() - shotStartedTime > Settings.shotTime) {
                        robotState = robotStates.DRIVING;
                    }

                } else {
                    feederV = 0;
                }

                break;

            case CLIMBING:
                updateClimberCount();
                feederV = 0;
                if(climbCounter == 1){
                    armPos = Settings.chainGrabArmPos;
                    wristPos = Settings.chainGrabWristpos; 
                    freeFeederControl();
                    manualClimberControl();
                } else if(climbCounter == 2){
                    if(thisRobot.teleopController.buttonPannel.getRawButton(Settings.climberPrepButton)){
                        armPos = Settings.climbArmPos;
                        wristPos = Settings.climbWristpos;
                        freeFeederControl();
                    } else {
                        if(climberReachedMaxHeight()){
                            //feed out
                            buttonClimberControl();
                            feederV = Settings.feederScoreTrapVoltage;
                            
                        } else {
                            //auto climb
                            freeFeederControl();
                            balanceClimber();
                        }
                    }
                } else {
                    if(climbCounter > 3){
                        climbCounter = 2;
                    }
                    freeFeederControl();
                    buttonClimberControl();
                }

                ss = intakeVoltage = 0;

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

        if(robotState != robotStates.CLIMBING){
            thisRobot.climber.climberMotor1.set(0);
            thisRobot.climber.climberMotor2.set(0);

        }

        thisRobot.arm.setArmPosition(armPos);
        thisRobot.wrist.setWristPos(wristPos);

        //if first shimmy count, run note out unitl sensor isnt broken
        if (shimmyCount == 0) {
            if (thisRobot.shooter.intakeSensorSeesNote()) {
                thisRobot.shooter.setFeederVoltage(-Settings.shimmyInVoltage);
            } else {
                //when note is out, inc count, set the start dist, and shimmy in
                shimmyCount = 1;
                shimmyStartDist = thisRobot.shooter.getFeederPos();
                shimmyIn = true;
            }
        } else {
            //after first shimy, come in unitl sensor is hit, then go out shimmy dist
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
                if(needToFeedIn){
                    if(Math.abs(thisRobot.shooter.getFeederPos() - feedInStartDist) < Settings.feedInDist){
                        feederV = Settings.feederIntakeVoltage;
                    } else {
                        needToFeedIn = false;
                    }
                }
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

    public boolean climberReachedMaxHeight() {
        return (thisRobot.climber.climberMotor1.getEncoder().getPosition() + thisRobot.climber.climberMotor1.getEncoder().getPosition())/2 > Settings.climberDistance;
    }

    // manual control of the feeder
    public void freeFeederControl() {
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.runFeederInButton)) {
            feederV = Settings.feederIntakeVoltage;
        } else if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.runFeederOutButton)) {
            feederV = Settings.feederScoreAmpVoltage;
        }
    }

    // manual spit out of intake
    public void freeIntakeControl() {
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.intakeOutButton)) {
            intakeVoltage = -Settings.intakingVoltage;
        }
    }

    public void buttonClimberControl(){
        double roll = thisRobot.drivebase.gyro.getRoll();
        double climbPowerDelta = roll * 0.1;
         if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbDownButton)) {
            thisRobot.climber.climberMotor1.set(-1);
            thisRobot.climber.climberMotor2.set(1);
         } else if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbUpButton)) {
            thisRobot.climber.climberMotor1.set(1 + climbPowerDelta);
            thisRobot.climber.climberMotor2.set(-1 + climbPowerDelta);
        } else {
            manualClimberControl();
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
            needToFeedIn = true;
            feedInStartDist = thisRobot.shooter.getFeederPos();
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.ampPrepButton)) {
            robotState = robotStates.SCORE_AMP;
            needToFeedIn = true;
            feedInStartDist = thisRobot.shooter.getFeederPos();
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.drivingStateReturnButton)) {
            robotState = robotStates.DRIVING;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }
    }

    public void manualClimberControl() {
        //if climb up or down isnt pressed use the joystick to manually controll climbers
        if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbDownButton) == false &&
                thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbUpButton) == false) {

            double lc = thisRobot.teleopController.manualControlJoystick.getRawAxis(Settings.manualControlArmAxis);
            double rc = thisRobot.teleopController.manualControlJoystick.getRawAxis(Settings.manualControlWristAxis);
            // cm 2 is left climber
            // + roll is to the right
            // + roll means subtract from left clmber
            // - roll means add that negative roll to riht climber
            // lc + power is climb up
            // rc - power is climb up
            thisRobot.climber.climberMotor1.set(rc);
            thisRobot.climber.climberMotor2.set(-lc);
        } else {
            //if up or down is pressed, get the roll and run a P loop on the angle to keep robot level
            double roll = thisRobot.drivebase.gyro.getRoll();
            double climbPowerDelta = roll * 0.1;
            if (climbPowerDelta > 1) {
                climbPowerDelta = 1;
            }
            if (climbPowerDelta < -1) {
                climbPowerDelta = -1;
            }
            if (thisRobot.teleopController.buttonPannel.getRawButton(Settings.climbDownButton)) {
                thisRobot.climber.climberMotor1.set(-1);
                thisRobot.climber.climberMotor2.set(1);
            } else {
                thisRobot.climber.climberMotor1.set(1 + climbPowerDelta);
                thisRobot.climber.climberMotor2.set(-1 + climbPowerDelta);

            }

        }

    }

    public void balanceClimber(){
        double roll = thisRobot.drivebase.gyro.getRoll();
        double climbPowerDelta = roll * 0.1;
        if (climbPowerDelta > 1) {
            climbPowerDelta = 1;
        }
        if (climbPowerDelta < -1) {
            climbPowerDelta = -1;
        }
        thisRobot.climber.climberMotor1.set(1 + climbPowerDelta);
        thisRobot.climber.climberMotor2.set(-1 + climbPowerDelta);
    
    }

    // lerp dist to wrist angle plus overide 
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

    public int updateClimberCount(){
        
        if(thisRobot.teleopController.buttonPannel.getRawButtonPressed(Settings.climberPrepButton)){
            climbCounter = climbCounter + 1;
        }
        return climbCounter;
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
