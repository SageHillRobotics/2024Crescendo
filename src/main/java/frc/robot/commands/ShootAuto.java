package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class ShootAuto extends Command{
    private double startingTime;
    private Flywheel flywheel;
    public ShootAuto(Flywheel flywheel){
        this.flywheel = flywheel;
    }
    @Override
    public void initialize(){
        startingTime = System.currentTimeMillis();
    }
    @Override
    public void execute(){
        flywheel.shoot();
    }
    @Override
    public boolean isFinished(){
        return (System.currentTimeMillis() - startingTime > 200);
    }
    @Override
    public void end(boolean interrupted){
        flywheel.stopIndex();
        flywheel.stopFlywheel();
    }
}
