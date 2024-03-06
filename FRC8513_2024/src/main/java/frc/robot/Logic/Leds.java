package frc.robot.Logic;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;


    public class Leds {
    public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(300);
    public AddressableLED m_led = new AddressableLED(4);
    private Robot thisRobot = new Robot();

        public Leds(Robot thisRobot5){
            m_led = new AddressableLED(4);
            thisRobot = thisRobot5;
        
        m_ledBuffer = new AddressableLEDBuffer(300);
        m_led.setLength(m_ledBuffer.getLength());
       
    
        m_led.setData(m_ledBuffer);
        m_led.start();


        if ((Timer.getMatchTime() <= 30) && (Timer.getMatchTime() >= 10)){
            if (Timer.getMatchTime() % 2 == 0){
                 for (int i = 0; i < m_ledBuffer.getLength(); i++){
                m_ledBuffer.setRGB(i, 255, 255, 255);
            }
        }   else {
            for (int i = 0; i < m_ledBuffer.getLength(); i++){
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }
        else if ((Timer.getMatchTime() <= 10)){
            if (Timer.getMatchTime() % 2 == 0){
                 for (int i = 0; i < m_ledBuffer.getLength(); i++){
                m_ledBuffer.setRGB(i, 255, 0, 0);
            }
        }   else {
            for (int i = 0; i < m_ledBuffer.getLength(); i++){
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
        }
        else if (thisRobot.stateMachine.robotState == StateMachine.robotStates.DRIVING){
            for (int i = 0; i<m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 117, 62, 189);
        }
    }
        else if(thisRobot.stateMachine.robotState == StateMachine.robotStates.INTAKING){
             for (int i = 0; i<m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 255, 9);
        }
    }
        else if (thisRobot.stateMachine.robotState == StateMachine.robotStates.SPEEDING_UP_SHOOTER_SPEAKER){
         for (int i = 0; i<m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 196);
        }   
    }
        else {
            for (int i = 0; i<m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 43, 0, 255);
        }   
    }


}
}