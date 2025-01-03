// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(4.5, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(5, 0, 0);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
      public static final class Vision {
      public static final Translation3d CAM_TRANSLATION = new Translation3d(Units.inchesToMeters(-3.0), 
                                                                        Units.inchesToMeters(12.0), 
                                                                        Units.inchesToMeters(26.75));
      public static final Rotation3d CAM_ROTATION =  new Rotation3d(0, Units.degreesToRadians(-19.18), Units.degreesToRadians(-17.38));

  }
}



