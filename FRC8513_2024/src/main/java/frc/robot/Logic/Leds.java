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

        m_ledBuffer = new AddressableLEDBuffer(100);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void updateLeds() {

        if(thisRobot.isEnabled()){
            if(thisRobot.stateMachine.robotState == robotStates.DRIVING){
                changeLedColor(30, 200, 30);
            }
            if(thisRobot.stateMachine.robotState == robotStates.INTAKING){
                changeLedColor(200, 30, 30);
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
                if(thisRobot.stateMachine.climbCounter == 0){
                    changeLedColor(255, 0, 0);
                }
                if(thisRobot.stateMachine.climbCounter == 1){
                    changeLedColor(0, 0, 255);
                }
                if(thisRobot.stateMachine.climbCounter == 2){
                    changeLedColor(0, 255, 0);
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

}
