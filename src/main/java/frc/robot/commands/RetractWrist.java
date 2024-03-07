package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Wrist;

public class RetractWrist extends Command{
    private final Wrist wrist;
    public RetractWrist(Wrist wrist){
        this.wrist = wrist;
        addRequirements(wrist);
    }
    @Override
    public void execute(){
        wrist.retract();
    }
}
