package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.StateMachine.robotStates;

public class AutoController {

    Robot thisRobot;
    int autoStep;
    public autoRoutines autoRoutine = autoRoutines.DO_NOTHING;
    public double autoStartTime;

    public AutoController(Robot thisRobot_) {
        thisRobot = thisRobot_;
    }

    public void autoInit() {
        // get dashboard auto selector value
        autoRoutine = autoRoutines.valueOf(thisRobot.dashboard.autoSelector.getSelected());
        SmartDashboard.putString("AutoRoutine", autoRoutine.toString());

        // reset auto vars
        autoStartTime = Timer.getFPGATimestamp();
        autoStep = 0;

        // check DS to get what alliance we are on for flipping paths
        thisRobot.updateAlliance();

        // force shooter on
        thisRobot.stateMachine.forceShooterOn = true;
    }

    public void autoPeriodic() {

        switch (autoRoutine) {
            case DO_NOTHING:
                switch (autoStep) {
                    case 0:
                        thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        thisRobot.drivebase.swerveDrive.lockPose();

                        autoStep = 5;
                        break;

                    case 5:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;

            case Amp_P3:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("AmpStartToNote3ToAmpShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;

            case Mid_P2:
                // simulation looks good
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                        }
                        break;
                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;

            case Source_P1:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceStartToNote1ToSourceShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                        }
                        break;
                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;

            case Source_P12:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceStartToNote1ToSourceShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("SourceShotToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case Amp_P321:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("AmpStartToNote3ToAmpShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("AmpSideShotToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote1AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 80:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;

            case Mid_P123:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote1ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("MiddleShotToNote2AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote3AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 80:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;

            case Mid_P231:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("MiddleShotToNote3AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote1AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 80:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;
            
            case Mid_P213:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("MiddleShotToNote1AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote3AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 80:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;


            case Mid_P321:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote3ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("MiddleShotToNote2AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote1AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 80:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;
            case Source_P123:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceStartToNote1ToSourceShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("SourceShotToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 55:
                        thisRobot.drivebase.initPath("MiddleShotToNote3AndBack", thisRobot.onRedAlliance);
                        autoStep = 65;

                    case 65:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 70;
                        }

                        break;
                    case 70:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 75;

                    case 75:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 80;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 80:
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;

            case Amp_P34:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("AmpStartToNote3ToAmpShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("AmpSideShotToNote4BackToShot", thisRobot.onRedAlliance);
                        autoStep = 40;
                        Settings.usePhoton = false;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;
                        
                        Settings.usePhoton = true;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case Amp_P35:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("AmpStartToNote3ToAmpShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("AmpSideShotToNote5BackToShot", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case Mid_P23:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("MiddleShotToNote3AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case Mid_P21:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("MiddleShotToNote1AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case Amp_P32:
                // good in simulation
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("AmpStartToNote3ToAmpShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("AmpSideShotToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case Source_P18:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceStartToNote1ToSourceShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("SourceShotToNote8AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case Source_P17:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("SourceStartToNote1ToSourceShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("SourceShotToNote7AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case Mid_P26:
                // works in sim
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("MiddleStartToNote2ToMidShot", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 10;

                    case 10:
                        thisRobot.stateMachine.updateRobotState();
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 15;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                            thisRobot.drivebase.trajStartTime = Timer.getFPGATimestamp();
                        }
                        break;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 20;
                        }
                        break;

                    case 20:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        thisRobot.stateMachine.lastStateChangeTime = Timer.getFPGATimestamp();
                        autoStep = 25;

                    case 25:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 30;
                            thisRobot.stateMachine.forceRobotState(robotStates.INTAKING);
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.initPath("MiddleShotToNote6AndBack", thisRobot.onRedAlliance);
                        autoStep = 40;

                    case 40:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 45;
                        }
                        break;

                    case 45:
                        thisRobot.stateMachine.forceRobotState(robotStates.SPEEDING_UP_SHOOTER_SPEAKER);
                        autoStep = 50;

                    case 50:
                        thisRobot.drivebase.aimAtGoal();
                        if (thisRobot.stateMachine.robotState == robotStates.DRIVING || timePassedInState(1)) {
                            autoStep = 55;
                            thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        }
                        break;

                    case 55:
                        thisRobot.stateMachine.forceShooterOn = false;
                }
                break;

            case _XTESTINGACCURACY:
                switch (autoStep) {
                    case 0:
                        thisRobot.stateMachine.forceShooterOn = false;
                        Settings.usePhoton = false;
                        thisRobot.drivebase.initPath("TestingPath", thisRobot.onRedAlliance);
                        autoStep = 5;

                    case 5:
                        thisRobot.stateMachine.forceRobotState(robotStates.DRIVING);
                        autoStep = 15;

                    case 15:
                        thisRobot.drivebase.followPath();
                        if (thisRobot.drivebase.isPathOver()) {
                            autoStep = 30;
                        }
                        break;

                    case 30:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        thisRobot.stateMachine.forceShooterOn = false;
                        break;
                }
                break;

            default:
                break;
        }

        thisRobot.stateMachine.updateRobotState();
        thisRobot.updateAllSubsystemMotorPower();
    }

    public double autoElapsedTime() {
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    //if in simulation, use t. if on real field, move on from shot if it takes more than 2s to get up to speed
    public boolean timePassedInState(double t) {
        if(Robot.isSimulation()){
            return ((Timer.getFPGATimestamp() - thisRobot.stateMachine.lastStateChangeTime) > t);
        } else {
            return (Timer.getFPGATimestamp() - thisRobot.stateMachine.lastStateChangeTime) > 2;
        }
    }

    // _X means not made yet

    public enum autoRoutines {
        DO_NOTHING,
        Amp_P3,
        Amp_P32,
        Amp_P34,
        Amp_P35,
        Amp_P321,
        Mid_P2,
        Mid_P21,
        Mid_P23,
        Mid_P26,
        Mid_P123,
        Mid_P213,
        Mid_P231,
        Mid_P321,
        Source_P1,
        Source_P12,
        Source_P17,
        Source_P18,
        Source_P123,
        _XTESTINGACCURACY

    }
}
