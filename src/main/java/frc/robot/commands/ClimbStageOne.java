package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class ClimbStageOne extends SequentialCommandGroup{
    public ClimbStageOne(Wrist wrist, Elevator elevator){
        addCommands(elevator.runOnce(elevator::ampPosition), wrist.runOnce(wrist::climbPosition));
    }
}

