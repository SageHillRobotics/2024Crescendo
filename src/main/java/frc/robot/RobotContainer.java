package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final CommandXboxController armController = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int cwButton = 8;
    private final int ccwButton = 7;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 2);
    private final JoystickButton robotCentric = new JoystickButton(driver, 12);
    private final JoystickButton sourceButton = new JoystickButton(driver, 5);
    private final JoystickButton speakerButton = new JoystickButton(driver, 3);
    private final JoystickButton ampButton = new JoystickButton(driver, 4);
    private final JoystickButton awayButton = new JoystickButton(driver, 6);
    private final Trigger retractShit = armController.y();
    private final Trigger spinFlywheel = armController.rightBumper();
    private final Trigger shoot = armController.rightTrigger();
    private final Trigger intakeNote = armController.x();
    private final Trigger retractElevator = armController.b();
    private final Trigger extendElevator = armController.a();
    

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final Intake intake = new Intake();
    public final Elevator elevator = new Elevator();
    public final Flywheel flywheel = new Flywheel();
    public final Wrist wrist = new Wrist();
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> driver.getRawButton(cwButton),
                () -> driver.getRawButton(ccwButton),
                () -> robotCentric.getAsBoolean()
            )
        );
        //intake.setDefaultCommand(new RetractIntake(intake));
        //wrist.setDefaultCommand(new RetractWrist(wrist));
        

        // Configure the button bindings
        configureButtonBindings();
        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        Optional<Alliance> ally = DriverStation.getAlliance();
        double invertIfRed = -1.0;
        if (ally.get() == Alliance.Red) {
            invertIfRed = 1.0;
        }
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        sourceButton.onTrue(new AnglePID(s_Swerve, 
        (60 * invertIfRed),
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis))
        );
        speakerButton.onTrue(new AnglePID(s_Swerve, 
        180,
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis))
        );
        ampButton.onTrue(new AnglePID(s_Swerve, 
        (-90 * invertIfRed),
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis))
        );
        awayButton.onTrue(new AnglePID(s_Swerve, 
        0,
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis))
        );
        retractShit.onTrue(new SequentialCommandGroup(
                                                      flywheel.runOnce(flywheel::stopFlywheel),
                                                      flywheel.runOnce(flywheel::stopIndex),
                                                      wrist.runOnce(wrist::retract),
                                                      new WaitUntilCommand(wrist::atHomePosition)));
                                                      //intake.runOnce(intake::retract)));
        
        spinFlywheel.onTrue(new InstantCommand(() -> flywheel.spinFlywheel()));
        shoot.onTrue(new Shoot(flywheel));
        intakeNote.onTrue(new SequentialCommandGroup(
                                            //intake.runOnce(intake::extend),
                                            flywheel.runOnce(flywheel::intake),
                                            new WaitUntilCommand(intake::atExtendedPosition), 
                                            wrist.runOnce(wrist::intakePosition), 
                                            new WaitUntilCommand(wrist::atIntakePosition),
                                            new IntakeNote(intake, flywheel), 
                                            wrist.runOnce(wrist::retract),
                                            new WaitUntilCommand(wrist::atHomePosition)
                                            //intake.runOnce(intake::retract)
                                            ));
        extendElevator.onTrue(new SequentialCommandGroup(
                            new InstantCommand(() -> elevator.ampPosition())
                            //wrist.runOnce(wrist::ampPosition)
                            ));
        retractElevator.onTrue(new SequentialCommandGroup(
                                //wrist.runOnce(wrist::retract),
                                //new WaitUntilCommand(wrist::atHomePosition),
                                new InstantCommand(() -> elevator.retract())
                                
                                ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new Auto(s_Swerve, "pathplanner/generatedJSON/New Path.wpilib.json");
    }
}
