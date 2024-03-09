package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private final Trigger retractEverything = armController.y();
    private final Trigger spinFlywheel = armController.rightBumper();
    private final Trigger shoot = armController.rightTrigger();
    private final Trigger intakeNote = armController.povUp();
    private final Trigger retractElevator = armController.b();
    private final Trigger extendElevator = armController.a();
    private final Trigger eject = armController.leftTrigger();
    private final Trigger climb = armController.leftBumper();
    private final Trigger floorIntake = armController.x();
    

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final Intake intake = new Intake();
    public final Elevator elevator = new Elevator();
    public final Flywheel flywheel = new Flywheel();
    public final Wrist wrist = new Wrist();
    public final LEDs leds = new LEDs();
    private SendableChooser<Command> autoChooser;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        
        
        NamedCommands.registerCommand("intake", new FloorIntake(elevator, intake, flywheel, wrist));
        NamedCommands.registerCommand("spinFlywheel", new InstantCommand(() -> flywheel.spinFlywheel()));
        NamedCommands.registerCommand("shoot", new Shoot(flywheel));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        


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
        leds.setDefaultCommand(new LEDCommand(leds, elevator));

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
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red){
                invertIfRed = 1.0;
            }
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
        retractEverything.onTrue(new SequentialCommandGroup(
                                                            flywheel.runOnce(flywheel::stopFlywheel),
                                                            flywheel.runOnce(flywheel::stopIndex),
                                                            wrist.runOnce(wrist::retract),
                                                            new WaitUntilCommand(wrist::atHomePosition),
                                                            new ParallelCommandGroup(intake.runOnce(intake::retract), elevator.runOnce(elevator::retract))));
        
        spinFlywheel.onTrue(new InstantCommand(() -> flywheel.spinFlywheel()));
        shoot.onTrue(new Shoot(flywheel));
        //intakeNote.onTrue(new IntakeCommand(elevator, intake, flywheel, wrist));
        intakeNote.onTrue(new SequentialCommandGroup(
                                                    flywheel.runOnce(flywheel::intake),
                                                    new ParallelCommandGroup(wrist.runOnce(wrist::sourcePosition), elevator.runOnce(elevator::sourcePosition)),
                                                    new WaitUntilCommand(wrist::atIntakePosition),
                                                    new SpinIntake(intake, flywheel), 
                                                    new ParallelCommandGroup(wrist.runOnce(wrist::retract), elevator.runOnce(elevator::retract))

        ));
        extendElevator.onTrue(new SequentialCommandGroup(
                                                            elevator.runOnce(elevator::ampPosition),
                                                            wrist.runOnce(wrist::ampPosition),
                                                            flywheel.runOnce(flywheel::spinFlywheel)
                                                            ));
        retractElevator.onTrue(new SequentialCommandGroup(
                                                            flywheel.runOnce(flywheel::stopFlywheel),
                                                            flywheel.runOnce(flywheel::stopIndex),
                                                            wrist.runOnce(wrist::retract),
                                                            new WaitUntilCommand(wrist::canRetract),
                                                            new InstantCommand(() -> elevator.retract()),
                                                            intake.runOnce(intake::retract)

                                                            ));
        eject.onTrue(intake.runOnce(intake::eject));
        climb.toggleOnTrue(new Climb(wrist, elevator));
        floorIntake.onTrue(new FloorIntake(elevator, intake, flywheel, wrist));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();

    }
}
