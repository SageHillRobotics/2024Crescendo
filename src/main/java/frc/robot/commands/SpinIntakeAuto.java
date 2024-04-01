package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;

public class SpinIntakeAuto extends Command{
    private Intake intake;
    private Flywheel flywheel;
    private double startingTime;
    public SpinIntakeAuto(Intake intake, Flywheel flywheel){
        this.intake = intake;
        this.flywheel = flywheel;
        addRequirements(intake);
        addRequirements(flywheel);
    }
    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();
        intake.spinIntakeMotor();
    }
    @Override
    public boolean isFinished(){
         return ((flywheel.getCurrent() > 25) || System.currentTimeMillis() - startingTime > 1000);
    }
    @Override
    public void end(boolean interrupted){
        intake.stopIntakeMotor(); 
        flywheel.stopFlywheel();
    }
}
