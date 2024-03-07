package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;

public class LEDCommand extends Command{
    private LEDs led;
    private Elevator elevator;
    public LEDCommand(LEDs led, Elevator elevator){
        this.led = led;
        this.elevator = elevator;
        addRequirements(led);
    }
    @Override
    public void execute(){
        if (elevator.getPosition() > 0.1){
            led.setRed();
        }
        else{
            led.setGreen();
        }
    }
}
