// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(30, 29, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class ElevatorConstants
  {
    public static final double ELEV_KP = 0.1;
    public static final double ELEV_KI = 0;
    public static final double ELEV_KD = 0.1;
    
    public static final double ELEV_START = 0.1;
    public static final double ELEV_LVL1 = 0.2;
    public static final double ELEV_LVL2 = 0.3;
    public static final double ELEV_LVL3 = 0.4;
    public static final double ELEV_CLIMB = 0.5;

    public static final double ELEV_UP = 0.1;
    public static final double ELEV_DOWN = -0.1;

  }

  public static final class PivotConstants
  {
    public static final double PIVOT_KP = 0.1;
    public static final double PIVOT_KI = 0;
    public static final double PIVOT_KD = 0.1;

    public static final double PIVOT_INTAKE = 0.1;
    public static final double PIVOT_MID_SCORE = 0.2;
    public static final double PIVOT_HIGH_SCORE = 0.3;

    public static final double PIVOT_UP_SPEED = 0.1;
    public static final double PIVOT_DOWN_SPEED = -0.1;

  }

  public static final class FunnelConstants
  {
    public static final double FUNNEL_SPIT_SPEED = 0.1;
  }

  public static final class AlgaeConstants
  {
    public static final double ALGAE_SLURP_SPEED = 0.1;
  }

}
