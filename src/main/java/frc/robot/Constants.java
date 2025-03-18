// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  public static final double LOW_MAX_SPEED = Units.feetToMeters(3.25);
  public static final boolean elevatorMotorReversed = false;
  public static final boolean wristRotationMotorReversed = true; // Changed wrist rotation motor to be false,
                                                                 // originally true.

  public static final boolean coralWheelMotorReversed = false;
  public static final boolean algaeWheelMotorReversed = true;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class ElevatorConstants {

    public static double kDt = 0.02;
    public static double kMaxVelocity = 1.75;
    public static double kMaxAcceleration = 0.75;
    public static double kP = 1.3;
    public static double kI = 0.0;
    public static double kD = 0.7;
    public static double kS = 1.1;
    public static double kG = 1.2;
    public static double kV = 1.3;
    public static int intakeEncoderCounts = 50;
    public static int L1EncoderCounts = 0;
    public static int L2EncoderCounts = 90;
    public static int L3EncoderCounts = 232;
    public static int L4EncoderCounts = 430;
  }

  public static class ManipulatorConstants {
    public static int wristLowAngle = -11;
    public static int wristHighAngle = -11; // TODO: might need to have a different angle for L4 scoring
    public static int wristIntakeAngle = 0;
    public static int wristStartupAngle = 0;
    public static double wristkP = 0;
    public static double wristkI = 0;
    public static double wristkD = 0;
    public static double wristkS = -0.01;
    public static double wristkG = 0.68;
    public static double wristkV = 0.88;
    public static double wristMaxVelocity = 0;
    public static double wristMaxAcceleration = 0;
    public static final ArmFeedforward armFeedforward = new ArmFeedforward(wristkS, wristkG, wristkV);
  }

  public static class VisionConstants {
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

  }
}
