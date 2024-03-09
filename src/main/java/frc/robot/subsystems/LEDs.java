package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    public AddressableLED topElevator;
    public AddressableLEDBuffer ledBuffer;
    public LEDs(){
        topElevator = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(152);
        topElevator.setLength(ledBuffer.getLength());
        topElevator.setData(ledBuffer);
        topElevator.start();
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
}
