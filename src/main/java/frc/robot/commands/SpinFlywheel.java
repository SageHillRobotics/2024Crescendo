package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class SpinFlywheel extends Command{
    private Flywheel flywheel;
    public SpinFlywheel(Flywheel flywheel){
        this.flywheel = flywheel;
    }
    @Override
    public void execute(){
        flywheel.spinFlywheel();
    }
    @Override
    public void end(boolean interrupted){
        flywheel.stopFlywheel();
    }

}
