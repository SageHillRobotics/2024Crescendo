package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ToggleElevatorExtension extends Command{
    private final Elevator elevator;
    public ToggleElevatorExtension(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void execute(){
        elevator.extendPID();
    }
    @Override
    public void end(boolean interrupted){
        elevator.retractPID();
    }
}
