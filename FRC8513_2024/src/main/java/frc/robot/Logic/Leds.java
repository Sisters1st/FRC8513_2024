package frc.robot.Logic;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Logic.StateMachine.robotStates;

public class Leds {
    public AddressableLEDBuffer m_ledBuffer;
    public AddressableLED m_led;
    private Robot thisRobot;

    public Leds(Robot thisRobot5) {
        m_led = new AddressableLED(5);
        thisRobot = thisRobot5;

        m_ledBuffer = new AddressableLEDBuffer(200);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void updateLeds() {

        if(thisRobot.isEnabled()){
            if(thisRobot.stateMachine.robotState == robotStates.DRIVING){
                changeLedColor(117, 62, 189);
            }
            if(thisRobot.stateMachine.robotState == robotStates.INTAKING){
                changeLedColor(255, 182, 55);
            }
            if(thisRobot.stateMachine.robotState == robotStates.SPEEDING_UP_SHOOTER_SPEAKER){
                changeLedColor(255, 0, 0);
            }
            if(thisRobot.stateMachine.robotState == robotStates.SHOOTING){
                if(thisRobot.stateMachine.robotInAllTHolds()){  
                    changeLedColor(0, 255, 34);
                } else {
                    changeLedColor(255, 0, 0);
                }
            }
            if(thisRobot.stateMachine.robotState == robotStates.CLIMBING){
                changeLedColor(117, 62, 189);
                while(Timer.getMatchTime() <= 10){
                    m_led.setLength((int) (10-Timer.getMatchTime()*100));
                    changeLedColor(117, 62, 189);
                }
            }
            
        } else {
            changeLedColor(0, 0, 0);
        }
    }

    private void changeLedColor(int r, int b, int g) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, b, g);
        }
        m_led.setData(m_ledBuffer);
    }

    private void blinkLedColor(int r, int b, int g) {
        if (Timer.getMatchTime() % 2 == 0) {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, r, b, g);
            }
            m_led.setData(m_ledBuffer);
        } else {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
            m_led.setData(m_ledBuffer);
        }
    }
}
