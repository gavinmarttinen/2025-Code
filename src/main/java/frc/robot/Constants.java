// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
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
  public static class ElevatorConstants
  {
    public static final int elevatorMotorID = 14; 
    public static final int elevatorMotor1ID = 15; 
   // public static final int elevatorEncoderID = 1;
    public static final int sensorID = 1;
    public static final double stowPosition = 9.0;
    public static final double intakePosition = 6.16; //5.09
    public static final double L1Position = 1.5;
    public static final double L2Position = 7.53; //6.96
    public static final double L3Position = 13;
    public static final double L4Position = 21.95;
    public static final double highestPosition = 22.67;
    public static final double kP = 2; //0.25
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.4;
    public static final int limitSwitchChannel = 0;
  }

  public static class ArmConstants {

    public static final int armMotorID = 16;
    public static final int armEncoderID = 1;
    public static final double armScoringPosition = 1;
    public static final double leftIntakePosition = 1;
    public static final double rightIntakePosition = 1;
    public static final double VerticalPosition = 0.681;
    public static final double preScoreRight = .018;
    public static final double preScoreLeft = .36;
  }

  public static class ClimberConstants {
    public static final int climberMotorID = 1;
    public static final int climberEncoderID = 1;
    public static final double climberMotorSpeed = .2;
  }

   public static final class Field{
    public static final Pose2d aprilTagSixLocation = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSevenLocation = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEightLocation = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineLocation = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTenLocation = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagElevenLocation = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwelveLocation = new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80),Rotation2d.fromDegrees(0));
    public static final Pose2d aprilTagThirteenLocation = new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20),Rotation2d.fromDegrees(0));
    public static final Pose2d aprilTagFourteenLocation = new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64),Rotation2d.fromDegrees(0));
    public static final Pose2d aprilTagFifteenLocation = new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39),Rotation2d.fromDegrees(0));
    public static final Pose2d aprilTagSixteenLocation = new Pose2d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15),Rotation2d.fromDegrees(0));
    public static final Pose2d aprilTagSeventeenLocation = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17),Rotation2d.fromDegrees(0)); //blue side
    public static final Pose2d aprilTagEighteenLocation = new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagNineteenLocation = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagTwentyLocation = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagTwentyOneLocation = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagTwentyTwoLocation = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17),Rotation2d.fromDegrees(0));//blue side
  }
}
