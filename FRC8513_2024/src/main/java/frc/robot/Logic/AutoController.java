package frc.robot.Logic;


import frc.robot.Robot;

public class AutoController {

    Robot thisRobot;
    int autoStep;


    public AutoController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void autoInit(){
        autoStep = 0;
        thisRobot.drivebase.initPath("SillyPath");
    }

    public void autoPeriodic(){        
        thisRobot.drivebase.followPath();
    }
}
