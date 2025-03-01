// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsytem;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverController = new CommandPS5Controller(0);
  final CommandPS5Controller operatorController = new CommandPS5Controller(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  private final ArmSubsytem armSubsystem = new ArmSubsytem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  //private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private Command score(double position, double side) {
    return new SequentialCommandGroup(
      Commands.run(()-> elevatorSubsystem.setMotorPosition(position), elevatorSubsystem), 
      Commands.run(()->armSubsystem.setMotorPosition(side), armSubsystem),
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition),elevatorSubsystem));
  }

  private final Command scoreLeftL2 = score(ElevatorConstants.L2Position,ArmConstants.leftIntakePosition);
  private final Command scoreRightL2 = score(ElevatorConstants.L2Position,ArmConstants.rightIntakePosition);
  private final Command scoreLeftL3 = score(ElevatorConstants.L3Position,ArmConstants.leftIntakePosition);
  private final Command scoreRightL3 = score(ElevatorConstants.L3Position,ArmConstants.rightIntakePosition);
  private final Command scoreLeftL4 = score(ElevatorConstants.L4Position,ArmConstants.leftIntakePosition);
  private final Command scoreRightL4 = score(ElevatorConstants.L4Position,ArmConstants.rightIntakePosition);

  private final Command intakeCoral = new SequentialCommandGroup(
  Commands.run(()-> armSubsystem.setMotorPosition(ArmConstants.leftIntakePosition),armSubsystem),
  Commands.run(()-> elevatorSubsystem.intakeCoral()),
  Commands.run(()-> armSubsystem.setMotorPosition(ArmConstants.VerticalPosition)));

   private final SendableChooser<Command> autoChooser;
    // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
  //                                                                () -> -MathUtil.applyDeadband(driverController.getLeftY(),
  //                                                                                              OperatorConstants.LEFT_Y_DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(driverController.getLeftX(),
  //                                                                                              OperatorConstants.DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(driverController.getRightX(),
  //                                                                                              OperatorConstants.RIGHT_X_DEADBAND),
  //                                                                driverController.getHID()::getYButtonPressed,
  //                                                                driverController.getHID()::getAButtonPressed,
  //                                                                driverController.getHID()::getXButtonPressed,
  //                                                                driverController.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * 1,
                                                                () -> driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis(()->driverController.getRightX()*-1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

   SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> driverController.getLeftY() * .5,
                                                            () -> driverController.getLeftX() * .5)
                                                        .withControllerRotationAxis(()->driverController.getRightX()*-1)
                                                        .deadband(OperatorConstants.DEADBAND)
                                                        .scaleTranslation(1)
                                                        .allianceRelativeControl(true);

                                                        

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                                                                                             driverController::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverController.getLeftY(),
                                                                   () -> -driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> driverController.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

                                                                     SwerveInputStream driveToLeftReefPost = SwerveInputStream.of(drivebase.getSwerveDrive(), ()->drivebase.getClosestReefPostLeftXDistance(), ()->drivebase.getClosestReefPostLeftYDistance());

SwerveInputStream driveToRightReefPost = SwerveInputStream.of(drivebase.getSwerveDrive(), ()->drivebase.getClosestReefPostRightXDistance(), ()->drivebase.getClosestReefPostRightYDistance());

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
      // Register Named Commands
      NamedCommands.registerCommand("L4Height", 
      Commands.runOnce(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L4Position),elevatorSubsystem).withTimeout(2));

      NamedCommands.registerCommand("IntakeHeight", 
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.intakePosition),elevatorSubsystem).withTimeout(2));

      NamedCommands.registerCommand("StowHeight", 
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition),elevatorSubsystem).withTimeout(2));

      NamedCommands.registerCommand("PreScoreLeft", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.preScoreLeft),armSubsystem).withTimeout(1.5));
      
      NamedCommands.registerCommand("PreScoreRight", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.preScoreRight),armSubsystem).withTimeout(1.5));

      NamedCommands.registerCommand("VerticalPosition", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition),armSubsystem).withTimeout(2));

      NamedCommands.registerCommand("driveToRightReefPost", drivebase.driveFieldOriented(driveToRightReefPost.withControllerRotationAxis(()-> 
      drivebase.getClosestAprilTagRotationPID())).until(()->drivebase.isInDistanceToleranceRight()));
    
      NamedCommands.registerCommand("driveToLeftReefPost", drivebase.driveFieldOriented(driveToLeftReefPost.withControllerRotationAxis(()-> 
      drivebase.getClosestAprilTagRotationPID())).until(()->drivebase.isInDistanceToleranceLeft()));

      // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
   autoChooser = AutoBuilder.buildAutoChooser("W1C1");
   SmartDashboard.putData("Auto Chooser", autoChooser);
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
    elevatorSubsystem.setDefaultCommand(Commands.run(()->{
      elevatorSubsystem.setMotor(MathUtil.applyDeadband(-operatorController.getLeftY(),0.05));
    }, elevatorSubsystem));

    armSubsystem.setDefaultCommand(Commands.run(()->{
      var leftCtrl = Math.min(MathUtil.applyDeadband(operatorController.getRawAxis(4),0.01), 0.0);
      var rotLeft = .4*Math.pow(leftCtrl, 2);
      var rightCtrl = Math.min(MathUtil.applyDeadband(operatorController.getRawAxis(3),0.01),0.0);
      var rotRight = .4*Math.pow(rightCtrl,2);
      SmartDashboard.putNumber("RotLeft", rotLeft);
      SmartDashboard.putNumber("LeftCtrl", leftCtrl);
      SmartDashboard.putNumber("RotRight", rotRight);
      armSubsystem.setMotor(rotLeft - rotRight);
    }, armSubsystem));
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                                

    if (Robot.isSimulation())
    {
     // driverController.square().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    //   driverController.cross().whileTrue(drivebase.sysIdDriveMotorCommand());
    //   driverController.triangle().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverController.circle().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    //   driverController.square().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //  //driverController.back().whileTrue(drivebase.centerModulesCommand());
     // driverController.leftBumper().onTrue(Commands.none());
     // driverController.rightBumper().onTrue(Commands.none());
    } else
    {
       driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
       operatorController.square().onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.intakePosition),elevatorSubsystem));
       operatorController.triangle().onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L3Position),elevatorSubsystem));
       operatorController.circle().onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L4Position),elevatorSubsystem));
       operatorController.cross().onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition),elevatorSubsystem));
       operatorController.button(10).onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L2Position),elevatorSubsystem));
       operatorController.button(9).whileTrue(Commands.run(()->elevatorSubsystem.resetEncoder(), elevatorSubsystem));
     //  operatorController.L1().whileTrue(Commands.run(()->climberSubsystem.setMotor(ClimberConstants.climberMotorSpeed),climberSubsystem)).whileFalse(Commands.run(()->climberSubsystem.stopMotor(),climberSubsystem));
      // operatorController.R1().whileTrue(Commands.run(()->climberSubsystem.setMotor(-ClimberConstants.climberMotorSpeed),climberSubsystem)).whileFalse(Commands.run(()->climberSubsystem.stopMotor(),climberSubsystem));



       operatorController.povDown().onTrue(Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition),armSubsystem)).onFalse(armSubsystem.getDefaultCommand());
       operatorController.povLeft().onTrue(Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.preScoreLeft),armSubsystem)).onFalse(armSubsystem.getDefaultCommand());
       operatorController.povRight().onTrue(Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.preScoreRight),armSubsystem)).onFalse(armSubsystem.getDefaultCommand());
     // driverController.R1().whileTrue(drivebase.driveFieldOriented(driveAngularVelocitySlow));
      driverController.L1().whileTrue(Commands.run(()->autoAlignToClosestAprilTag()));
      driverController.R1().whileTrue(Commands.run(()->autoAlignToClosestFeederStation()));
      driverController.L2().whileTrue(drivebase.driveFieldOriented(driveToLeftReefPost.withControllerRotationAxis(()->drivebase.getClosestAprilTagRotationPID())));
       driverController.R2().whileTrue(drivebase.driveFieldOriented(driveToRightReefPost.withControllerRotationAxis(()->drivebase.getClosestAprilTagRotationPID())));
     // driverController.R1().whileTrue(Commands.run(()->autoAlignToClosestAprilTagRight()));
      // driverController.L1().onTrue(Commands.runOnce(SignalLogger::start));
       //driverController.L2().onTrue(Commands.runOnce(SignalLogger::stop));
       //driverController.triangle().whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     // driverController.square().whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     // driverController.cross().whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
     // driverController.circle().whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
       //   driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //   driverController.b().whileTrue(
    //       drivebase.driveToPose(
    //           new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           );
    //   driverController.y().whileTrue(drivebase.aimAtSpeaker(2));
    //   driverController.start().whileTrue(Commands.none());
    //   driverController.back().whileTrue(Commands.none());
    //   driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverController.rightBumper().onTrue(Commands.none());
    
     }

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
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  private void autoAlignToClosestAprilTag(){
    drivebase.driveFieldOriented(drivebase.getTargetSpeeds(-driverController.getLeftY(), -driverController.getLeftX(),
    drivebase.getClosestAprilTagRotation()));
  }

  private void autoAlignToClosestFeederStation(){
    drivebase.driveFieldOriented(drivebase.getTargetSpeeds(-driverController.getLeftY(), -driverController.getLeftX(),
    drivebase.getClosestFeederStationRotation()));
  }
}
