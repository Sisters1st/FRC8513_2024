package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

public class AutoController {

    Robot thisRobot;

    public AutoController(Robot thisRobot_){
        thisRobot = thisRobot_;
    }

    public void autoInit(){

    }

    public void autoPeriodic(){
        PathPlannerPath path = PathPlannerPath.fromPathFile("StraightPath.path");
        path.getPoint(0);
    }
}
