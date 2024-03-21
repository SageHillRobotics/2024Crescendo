// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.FloorIntake;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpinIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
    public final Intake intake = new Intake();
    public final Elevator elevator = new Elevator();
    public final Flywheel flywheel = new Flywheel();
    public final Wrist wrist = new Wrist();
    public final LEDs leds = new LEDs();
    private SendableChooser<Command> autoChooser;
    

  // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandJoystick driver = new CommandJoystick(0);
    private final CommandXboxController armController = new CommandXboxController(1);


    private final Trigger cwButton = driver.button(7);
    private final Trigger ccwButton = driver.button(6);


    final Trigger shoot = driver.button(1);
    final Trigger zeroGyro = driver.button(2);
    final Trigger robotCentric = driver.button(12);
    final Trigger sourceButton = driver.button(5);
    final Trigger speakerButton = driver.button(3);
    final Trigger ampButton = driver.button(4);
    final Trigger awayButton = driver.button(6);

    private final Trigger retractEverything = driver.button(10);
    private final Trigger intakeNote = armController.povUp();
    private final Trigger retractElevator = armController.b();
    private final Trigger extendElevator = armController.a();
    private final Trigger eject = armController.leftTrigger();
    private final Trigger climb = armController.leftBumper();
    private final Trigger floorIntake = driver.button(9);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("intake", new FloorIntake(elevator, intake, flywheel, wrist));
    NamedCommands.registerCommand("spinFlywheel", new InstantCommand(() -> flywheel.spinFlywheel()));
    NamedCommands.registerCommand("shoot", new Shoot(flywheel));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();


    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    /*
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());
    */
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driver.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driver.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-driver.getRawAxis(2), 0.1));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    leds.setDefaultCommand(new LEDCommand(leds, elevator));
    //wrist.setDefaultCommand(wrist.runOnce(wrist::retract));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings()
    {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    zeroGyro.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    ampButton.whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                    new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                                ));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    shoot.onTrue(new SequentialCommandGroup(flywheel.runOnce(flywheel::spinFlywheel), new WaitCommand(1), new Shoot(flywheel)));
    retractEverything.onTrue(new SequentialCommandGroup(
                                                        flywheel.runOnce(flywheel::stopFlywheel),
                                                        flywheel.runOnce(flywheel::stopIndex),
                                                        wrist.runOnce(wrist::retract),
                                                        new WaitUntilCommand(wrist::atHomePosition),
                                                        new ParallelCommandGroup(intake.runOnce(intake::retract), elevator.runOnce(elevator::retract))));
    
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
                                                        flywheel.runOnce(flywheel::ampSpeed)
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
    floorIntake.onTrue(new FloorIntake(elevator, intake, flywheel, wrist));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setDriveMode()
    {
    //drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake)
    {
    drivebase.setMotorBrake(brake);
    }
}
