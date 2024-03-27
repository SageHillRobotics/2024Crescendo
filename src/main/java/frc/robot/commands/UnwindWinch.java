package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class UnwindWinch extends Command{
    Elevator elevator;
    public UnwindWinch(Elevator elevator){
        this.elevator = elevator;
    }
    @Override
    public void initialize(){
        elevator.unwindWinch();
    }
    @Override
    public void end(boolean interrupted){
        elevator.stop();
    }
}
