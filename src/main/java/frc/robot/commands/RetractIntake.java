package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RetractIntake extends Command{
    private final Intake intake;
    public RetractIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void execute(){
        intake.retract();
    }
}
