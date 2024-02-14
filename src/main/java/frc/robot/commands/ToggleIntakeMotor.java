package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ToggleIntakeMotor extends Command{
    private final Intake intake;
    public ToggleIntakeMotor(Intake intake){
        this.intake = intake;
    }

    @Override
    public void execute(){
        intake.spinIntakeMotor();
    }
    @Override
    public void end(boolean interruption){
        intake.stopIntakeMotor();
    }
}