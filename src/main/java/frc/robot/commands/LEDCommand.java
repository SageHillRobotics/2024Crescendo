package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class LEDCommand extends Command{
    private LEDs led;
    private Elevator elevator;
    private Intake intake;

    public LEDCommand(LEDs led, Elevator elevator, Intake intake){
        this.led = led;
        this.elevator = elevator;
        this.intake = intake;
        addRequirements(led);
    }
    @Override
    public void execute(){
        if (intake.getCurrent() > 10){
            led.flash();
        }
        else if (elevator.getPosition() > 0.1){
            led.setRed();
        }
        else{
            led.setGreen();
        }
    }
}
