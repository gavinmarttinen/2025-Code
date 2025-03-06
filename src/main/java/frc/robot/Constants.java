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
    public static final int sensorID = 1;
    public static final double stowPosition = 9.5;
    public static final double intakePosition = 6.16;
    public static final double L2Position = 7.53;
    public static final double L3Position = 13;
    public static final double L4Position = 21.95;
    public static final double highestPosition = 22.67;
    public static final double kS = 0.04087;
    public static final double kV = 0.12261;
    public static final double kA = 0.0033489;
    public static final double kP = 2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.4;
    public static final double CruiseVelocity = 80;
    public static final double Acceleration = 80;
    public static final double Jerk = 1600;
    public static final int limitSwitchChannel = 0;
  }

  public static class ArmConstants {

    public static final int armMotorID = 16;
    public static final int armEncoderID = 9;
    public static final double armScoringPosition = 1;
    public static final double leftIntakePosition = 1;
    public static final double rightIntakePosition = 1;
    public static final double VerticalPosition = 0.04;
    public static final double preScoreRight = .396;
    public static final double preScoreLeft = .61;
    public static final double P = 2.0;
    public static final double I = 0.0;
    public static final double D = 0;
  }

  public static class ClimberConstants {
    public static final int climberMotorID = 17;
    public static final int climberEncoderID = 46;
    public static final double climberMotorSpeed = 1;
  }

   public static final class Field{
    public static final Pose2d aprilTagOneLocation = new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwoLocation = new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20),Rotation2d.fromDegrees(0));//red side
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

    public static final Pose2d aprilTagSixLeftReef = new Pose2d(13.624, 2.731,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSixRightReef = new Pose2d(13.893, 2.895,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSevenLeftReef = new Pose2d(14.46,3.818,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSevenRightReef = new Pose2d(14.46,4.098,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEightLeftReef = new Pose2d(13.881,5.146,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEightRightReef = new Pose2d(13.653,5.292,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineLeftReef = new Pose2d(12.558,5.319,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineRightReef = new Pose2d(12.296,5.147,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTenLeftReef = new Pose2d(11.745,4.176,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTenRightReef = new Pose2d(11.745,3.841,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagElevenLeftReef = new Pose2d(12.268,2.873,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagElevenRightReef = new Pose2d(12.589,2.704,Rotation2d.fromDegrees(0));//red side

    public static final Pose2d aprilTagSeventeenLeftReef = new Pose2d(3.616, 2.858,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSeventeenRightReef = new Pose2d(3.929, 2.698,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEighteenLeftReef = new Pose2d(3.196,4.352,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEighteenRightReef = new Pose2d(3.094,3.888,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineteenLeftReef = new Pose2d(3.98,5.353,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineteenRightReef = new Pose2d(3.8259,5.1347,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyLeftReef = new Pose2d(5.4368,4.984,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyRightReef = new Pose2d(5.00,5.352,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyOneLeftReef = new Pose2d(5.914,3.862,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyOneRightReef = new Pose2d(5.872, 4.092,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyTwoLeftReef = new Pose2d(5.071,2.676,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyTwoRightReef = new Pose2d(5.312, 2.808,Rotation2d.fromDegrees(0));//red side

  }
  
}
