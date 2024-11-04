package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.LEDs;

public class LEDCommand extends Command {
    private LEDs led;
    private Elevator elevator;
    private Flywheel flywheel;
    private boolean blinking = false;
    private double blinkStartTime = 0.0;

    public LEDCommand(LEDs led, Elevator elevator, Flywheel flywheel){
        this.led = led;
        this.elevator = elevator;
        this.flywheel = flywheel;
        addRequirements(led);
    }

    @Override
    public void execute(){
        double currentTime = Timer.getFPGATimestamp();

        if (!blinking && flywheel.getCurrent() > 30){
            blinking = true;
            blinkStartTime = currentTime;
        }
        if (blinking){
            if (currentTime - blinkStartTime < 1.0){
                double elapsedTime = currentTime - blinkStartTime;
                if ((int)(elapsedTime / 0.1) % 2 == 0){
                    led.setGreen();
                } else {
                    led.setRed();
                }
            } else {
                blinking = false;
            }
        }

        if (!blinking){
            if (elevator.getPosition() > 0.1){
                led.setRed();
            } else {
                led.setGreen();
            }
        }
    }
}
