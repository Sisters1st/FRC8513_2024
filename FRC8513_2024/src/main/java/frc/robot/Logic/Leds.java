package frc.robot.Logic;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class Leds {
    public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(300);
    public AddressableLED m_led = new AddressableLED(4);
    private Robot thisRobot = new Robot();

    public Leds(Robot thisRobot5) {
        m_led = new AddressableLED(4);
        thisRobot = thisRobot5;

        m_ledBuffer = new AddressableLEDBuffer(300);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void updateLeds() {

        if (thisRobot.isAutonomous()) {
            m_led.stop();
        }

        if ((Timer.getMatchTime() <= 30) && (Timer.getMatchTime() >= 10)) {
            blinkLedColor(17, 0, 255);
        } else if ((Timer.getMatchTime() <= 10)) {
            blinkLedColor(255, 0, 0);
        } else if (thisRobot.stateMachine.robotState == StateMachine.robotStates.DRIVING) {
            changeLedColor(117, 62, 189);
        } else if (thisRobot.stateMachine.robotState == StateMachine.robotStates.INTAKING) {
            changeLedColor(0, 255, 9);
        } else {
            changeLedColor(43, 255, 0);

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
                m_ledBuffer.setRGB(i, 255, 255, 255);
            }
            m_led.setData(m_ledBuffer);
        }
    }
}
