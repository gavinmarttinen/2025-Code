// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    public static final double VerticalPosition = 0.0253;
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
    public static final double climberOutPosition = 0.863;
    public static final double climberInPosition = 0.320;
    public static final double P = 9;
    public static final double I = 0;
    public static final double D = 0;
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
    public static final Pose2d aprilTagTwelveLocation = new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagThirteenLocation = new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagFourteenLocation = new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagFifteenLocation = new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSixteenLocation = new Pose2d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15),Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSeventeenLocation = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17),Rotation2d.fromDegrees(0)); //blue side
    public static final Pose2d aprilTagEighteenLocation = new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagNineteenLocation = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagTwentyLocation = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagTwentyOneLocation = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0));//blue side
    public static final Pose2d aprilTagTwentyTwoLocation = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17),Rotation2d.fromDegrees(0));//blue side

    //Left Side of Robot Facing Reef
    public static final Pose2d aprilTagSixLeftReefRotLeft = new Pose2d(13.512, 2.831,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSixRightReefRotLeft = new Pose2d(13.6131, 2.997,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSevenLeftReefRotLeft = new Pose2d(14.393,3.82,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSevenRightReefRotLeft = new Pose2d(14.423,4.129,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEightLeftReefRotLeft = new Pose2d(14.032,4.932,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEightRightReefRotLeft = new Pose2d(13.776,4.997,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineLeftReefRotLeft = new Pose2d(12.657,5.399,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineRightReefRotLeft = new Pose2d(12.139,5.169,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTenLeftReefRotLeft = new Pose2d(11.577,4.264,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTenRightReefRotLeft = new Pose2d(11.7289,3.9515,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagElevenLeftReefRotLeft = new Pose2d(12.2333,3.034,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagElevenRightReefRotLeft = new Pose2d(12.428,2.903,Rotation2d.fromDegrees(0));//red side

    public static final Pose2d aprilTagSeventeenLeftReefRotLeft = new Pose2d(3.8088, 2.9222,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagSeventeenRightReefRotLeft = new Pose2d(3.929, 2.698,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEighteenLeftReefRotLeft = new Pose2d(3.196,4.352,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEighteenRightReefRotLeft = new Pose2d(3.094,3.888,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineteenLeftReefRotLeft = new Pose2d(3.98,5.353,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagNineteenRightReefRotLeft = new Pose2d(3.8093,5.1784,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyLeftReefRotLeft = new Pose2d(5.4449,5.0389,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyRightReefRotLeft = new Pose2d(5.00,5.352,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyOneLeftReefRotLeft = new Pose2d(5.914,3.862,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyOneRightReefRotLeft = new Pose2d(5.872, 4.092,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyTwoLeftReefRotLeft = new Pose2d(5.071,2.676,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagTwentyTwoRightReefRotLeft = new Pose2d(5.4634, 3.0288,Rotation2d.fromDegrees(0));//red side
    public static final List<Pose2d> leftReefLocations = List.of(aprilTagSixLeftReefRotLeft,aprilTagSevenLeftReefRotLeft,aprilTagEightLeftReefRotLeft,aprilTagNineLeftReefRotLeft,aprilTagTenLeftReefRotLeft,aprilTagElevenLeftReefRotLeft,aprilTagSeventeenLeftReefRotLeft,aprilTagEighteenLeftReefRotLeft,aprilTagNineteenLeftReefRotLeft,aprilTagTwentyLeftReefRotLeft,aprilTagTwentyOneLeftReefRotLeft,aprilTagTwentyTwoLeftReefRotLeft);
    public static final List<Pose2d> rightReefLocations = List.of(aprilTagSixRightReefRotLeft,aprilTagSevenRightReefRotLeft,aprilTagEightRightReefRotLeft,aprilTagNineRightReefRotLeft,aprilTagTenRightReefRotLeft,aprilTagElevenRightReefRotLeft,aprilTagSeventeenRightReefRotLeft,aprilTagEighteenRightReefRotLeft,aprilTagNineteenRightReefRotLeft,aprilTagTwentyRightReefRotLeft,aprilTagTwentyOneRightReefRotLeft,aprilTagTwentyTwoRightReefRotLeft);

    //Right Side of Robot Facing Reef
    public static final Pose2d aprilTagSixLeftReefRotRight = aprilTagSixLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagSixRightReefRotRight = aprilTagSixRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagSevenLeftReefRotRight = aprilTagSevenLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side //new Pose2d(14.474,3.968,Rotation2d.fromDegrees(0));
    public static final Pose2d aprilTagSevenRightReefRotRight = new Pose2d(14.46,4.098,Rotation2d.fromDegrees(0));//red side
    public static final Pose2d aprilTagEightLeftReefRotRight = aprilTagEightLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagEightRightReefRotRight = aprilTagEightRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagNineLeftReefRotRight = aprilTagNineLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagNineRightReefRotRight = aprilTagNineRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagTenLeftReefRotRight = aprilTagTenLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagTenRightReefRotRightt = aprilTagTenRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagElevenLeftReefRotRight = aprilTagElevenLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagElevenRightReefRotRight = aprilTagElevenRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side

    public static final Pose2d aprilTagSeventeenLeftReefRotRight = aprilTagSeventeenLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagSeventeenRightReefRotRight = aprilTagSeventeenRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagEighteenLeftReefRotRight = aprilTagEighteenLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagEighteenRightReefRotRight = aprilTagEighteenRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagNineteenLeftReefRotRight = aprilTagNineteenLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagNineteenRightReefRotRight = aprilTagNineteenRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagTwentyLeftReefRotRight = aprilTagTwentyLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagTwentyRightReefRotRightt = aprilTagTwentyRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagTwentyOneLeftReefRotRight = aprilTagTwentyOneLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagTwentyOneRightReefRotRight = aprilTagTwentyOneRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagTwentyTwoLeftReefRotRight = aprilTagTwentyTwoLeftReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side
    public static final Pose2d aprilTagTwentyTwoRightReefRotRight = aprilTagTwentyTwoRightReefRotLeft.transformBy(new Transform2d(0,0,new Rotation2d()));//red side

  }
  
}
