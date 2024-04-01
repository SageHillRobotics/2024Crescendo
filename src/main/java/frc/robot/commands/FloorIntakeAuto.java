package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;


public class FloorIntakeAuto extends SequentialCommandGroup{
    public FloorIntakeAuto(Elevator elevator, Intake intake, Flywheel flywheel, Wrist wrist){
    addCommands(intake.runOnce(intake::extend),
                    flywheel.runOnce(flywheel::intake),
                    new WaitUntilCommand(intake::atExtendedPosition), 
                    new ParallelCommandGroup(wrist.runOnce(wrist::intakePosition), elevator.runOnce(elevator::intakePosition)),
                    new WaitUntilCommand(wrist::atIntakePosition),
                    new SpinIntakeAuto(intake, flywheel), 
                    new ParallelCommandGroup(wrist.runOnce(wrist::retract), elevator.runOnce(elevator::retract)),
                    new WaitCommand(.3),
                    intake.runOnce(intake::retract));
    }
}
