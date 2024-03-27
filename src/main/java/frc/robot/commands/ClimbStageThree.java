package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class ClimbStageThree extends SequentialCommandGroup{
    public ClimbStageThree(Wrist wrist, Elevator elevator){
        addCommands(elevator.runOnce(elevator::retractWinch), new WaitUntilCommand(() -> elevator.getPosition() < 1.5), wrist.runOnce(wrist::trapPosition), new WaitUntilCommand(elevator::atWinchPosition), elevator.runOnce(elevator::stop));
    }
}
