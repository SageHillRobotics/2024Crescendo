package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    public AddressableLED topElevator;
    public AddressableLEDBuffer ledBuffer;
    public Timer timer;
    public LEDs(){
        topElevator = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(152);
        topElevator.setLength(ledBuffer.getLength());
        topElevator.setData(ledBuffer);
        topElevator.start();
        timer = new Timer();
    }
    public void setRed(){
        for (int i = 0; i < 152; i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
        topElevator.setData(ledBuffer);
    }
    public void setGreen(){
        for (int i = 0; i < 152; i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
        topElevator.setData(ledBuffer);
    }
    public void setOff(){
        for (int i = 0; i < 152; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        topElevator.setData(ledBuffer); 
    }
    public void flash(){
        timer.restart();
        if (timer.get() < 0.1){
            setGreen();
        }
        else if (timer.get() < 0.2){
            setOff();
        }
        else{
            timer.restart();
        }
    }
}
