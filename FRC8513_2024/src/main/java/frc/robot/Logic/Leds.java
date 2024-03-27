package frc.robot.Logic;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.StateMachine.robotStates;

public class Leds {
    public AddressableLEDBuffer m_ledBuffer;
    public AddressableLED m_led;
    private Robot thisRobot;
    int m_rainbowFirstPixelHue = 0;

    public Leds(Robot thisRobot5) {
        m_led = new AddressableLED(5);
        thisRobot = thisRobot5;

        m_ledBuffer = new AddressableLEDBuffer(100);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void updateLeds() {

        /* 

        last 20sec green, yellow 10 seconds, red last 5 if in climb or drive mode. 
        */


        if(thisRobot.isEnabled()){

            //if driving, green if sensor sees note, off if otherwise
            if(thisRobot.stateMachine.robotState == robotStates.DRIVING){
                if(thisRobot.intake.intakeSensorSeesNote() || thisRobot.shooter.feederSensorSeesNote()){
                    changeLedColor(0, 255, 0);
                } else {
                    changeLedColor(0, 0, 0);
                }

            }

            //if intakeing, red if camera doesnt see note, yellow if cam sees note, and green if either sensor sees note
            if(thisRobot.stateMachine.robotState == robotStates.INTAKING){
                if(thisRobot.intake.intakeSensorSeesNote() || thisRobot.shooter.feederSensorSeesNote()){
                    changeLedColor(0, 255, 0);
                } else {
                    if(LimelightHelpers.getTX(Settings.llName) == 0.0){
                        changeLedColor(255, 0, 0);
                    } else {
                        changeLedColor(255, 255, 0);
                    }
                }
            }

            //shooting state, if vision is out of date be red, if not ready to shoot yellow, ready to shoot is green
            if(thisRobot.stateMachine.robotState == robotStates.SHOOTING || thisRobot.stateMachine.robotState == robotStates.SPEEDING_UP_SHOOTER_SPEAKER){
                if(!thisRobot.drivebase.visionIsRecent()){
                    changeLedColor(255, 0,0);
                } else {
                    if(thisRobot.stateMachine.robotInAllTHolds()){
                        changeLedColor(0, 255, 0);
                    } else {
                        changeLedColor(255, 255, 0);
                    }
                }
            }

            if(thisRobot.stateMachine.robotState == robotStates.SCORE_AMP || thisRobot.stateMachine.robotState == robotStates.SCORE_AMP || thisRobot.stateMachine.robotState == robotStates.CLIMBING ){
                last20Sec();
            }
            
        } else {
            changeLedColor(0, 0, 0);
        }
    }

    public void last20Sec(){
        if(Timer.getMatchTime() < 5){
            changeLedColor(255, 0, 0);
        } else {
            if(Timer.getMatchTime() < 10){
              changeLedColor(255, 255, 0);
            } else {
                if(Timer.getMatchTime() < 20){
                    changeLedColor(0, 255, 0);
                } else {
                    changeLedColor(0, 0, 0);
                }
            }
        }
    }

    public void changeLedColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void updateRainbow(){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
          }
          // Increase by to make the rainbow "move"
          m_rainbowFirstPixelHue += 3;
          // Check bounds
          m_rainbowFirstPixelHue %= 180;
          m_led.setData(m_ledBuffer);
    }

    public boolean noteSeen(){
        return thisRobot.shooter.feederSensorSeesNote() || thisRobot.intake.intakeSensorSeesNote();
    }

}
