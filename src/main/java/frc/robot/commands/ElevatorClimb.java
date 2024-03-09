package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;

public class ElevatorClimb extends Command{
    private Elevator elevator;
    private final static double FINISHED_CLIMB_POSITION = 1.5;
    public ElevatorClimb(Elevator elevator){
        this.elevator = elevator;
    }
    @Override
    public void execute(){
        elevator.setSpeed(-0.03);
    }
    @Override
    public boolean isFinished(){
        return elevator.getPosition() < FINISHED_CLIMB_POSITION;
    }
    @Override
    public void end(boolean interrupted){
        elevator.setSpeed(-0.03);
    }
}
