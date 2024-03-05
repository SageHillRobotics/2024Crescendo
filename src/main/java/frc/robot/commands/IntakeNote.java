package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command{
    private Intake intake;
    private Flywheel flywheel;
    private Debouncer debouncer = new Debouncer(.1, Debouncer.DebounceType.kRising);
    public IntakeNote(Intake intake, Flywheel flywheel){
        this.intake = intake;
        this.flywheel = flywheel;
        addRequirements(intake);
        addRequirements(flywheel);
    }
    @Override
    public void initialize() {
        intake.spinIntakeMotor();
    }
    @Override
    public boolean isFinished(){
         return (debouncer.calculate(flywheel.getCurrent() > 5));
    }
    @Override
    public void end(boolean interrupted){
        intake.stopIntakeMotor(); 
        flywheel.stopFlywheel();
        flywheel.brake();
    }
}
