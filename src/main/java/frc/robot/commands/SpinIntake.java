package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;

public class SpinIntake extends Command{
    private Intake intake;
    private Flywheel flywheel;
    public SpinIntake(Intake intake, Flywheel flywheel){
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
         return ((flywheel.getCurrent() > 30));
    }
    @Override
    public void end(boolean interrupted){
        intake.stopIntakeMotor(); 
        flywheel.stopFlywheel();
    }
}
