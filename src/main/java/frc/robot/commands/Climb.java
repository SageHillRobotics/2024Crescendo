package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class Climb extends SequentialCommandGroup{
    public Climb(Wrist wrist, Elevator elevator){
        addCommands(wrist.runOnce(wrist::retract), new ElevatorClimb(elevator));
    }
}

