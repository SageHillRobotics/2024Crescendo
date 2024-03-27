package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private SwerveSubsystem s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private BooleanSupplier cwSup;
  private BooleanSupplier ccwSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);

  public TeleopSwerve(
      SwerveSubsystem s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      BooleanSupplier cwSup,
      BooleanSupplier ccwSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.cwSup = cwSup;
    this.ccwSup = ccwSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), 0.2));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.2));
    double rotationVal = 0;
    if (DriverStation.getAlliance().get() == Alliance.Red){
        translationVal *= -1;
        strafeVal *= -1;
    }
    if (cwSup.getAsBoolean() == true && ccwSup.getAsBoolean() == false){
      rotationVal = -0.8;
    }
    else if (ccwSup.getAsBoolean() == true && cwSup.getAsBoolean() == false){
      rotationVal = 0.8;
    }

    /* Drive */
    s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(s_Swerve.getMaximumVelocity()),
                        rotationVal * s_Swerve.getMaximumAngularVelocity(),
                        true);
  
  }
}