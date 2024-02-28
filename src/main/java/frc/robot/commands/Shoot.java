package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class Shoot extends Command{
    private Flywheel flywheel;
    public Shoot(Flywheel flywheel){
        this.flywheel = flywheel;
    }
    @Override
    public void execute(){
        flywheel.shoot();
    }

    @Override
    public void end(boolean interrupted){
        flywheel.stopIndex();
    }
}
