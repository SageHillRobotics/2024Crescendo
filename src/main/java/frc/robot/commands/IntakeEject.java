package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeEject extends Command{
    private double startTime;
    private Intake intake;
    public IntakeEject(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize(){
        startTime = System.currentTimeMillis();
        intake.eject();
    }
    @Override
    public boolean isFinished(){
        return System.currentTimeMillis() - startTime > 2;
    }
    @Override
    public void end(boolean interrupted){
        intake.stopIntakeMotor();
    }
}
