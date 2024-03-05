package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.AnglePID;
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
    private final JoystickButton sourceButton = new JoystickButton(driver, 6);
    private final JoystickButton speakerButton = new JoystickButton(driver, 3);
    private final JoystickButton ampButton = new JoystickButton(driver, 5);
    private final JoystickButton toggleIntakeMotor = new JoystickButton(driver, 5);
    private final Trigger toggleElevatorExtension = armController.a();
    private final Trigger spinFlywheel = armController.rightBumper();
    private final Trigger shoot = armController.rightTrigger();
    

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final Intake intake = new Intake();
    public final Elevator elevator = new Elevator();
    public final Flywheel flywheel = new Flywheel();
    
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        sourceButton.onTrue(new AnglePID(s_Swerve, 
        -45,
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis))
        );
        speakerButton.onTrue(new AnglePID(s_Swerve, 
        180,
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis))
        );
        ampButton.onTrue(new AnglePID(s_Swerve, 
        90,
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis))
        );
        toggleIntakeMotor.toggleOnTrue(new ToggleIntakeMotor(intake));
        toggleElevatorExtension.toggleOnTrue(new ToggleElevatorExtension(elevator));
        spinFlywheel.toggleOnTrue(new SpinFlywheel(flywheel));
        shoot.toggleOnTrue(new Shoot(flywheel));
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
