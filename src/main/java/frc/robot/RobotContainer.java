// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final SendableChooser<Command> autoChooser;

  private final Command intakeOutCommand = new ParallelCommandGroup(
    Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition),elevatorSubsystem).until(()->elevatorSubsystem.elevatorAtSetpoint()),
    Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition),armSubsystem).until(()->armSubsystem.armAtSetpoint()),
    Commands.run(()->intakeSubsystem.deployIntake(),intakeSubsystem).until(()->intakeSubsystem.isCoralDetectedAndPivotAtSetpoint())); //end of parallel

  private final Command intakeInCommand = new SequentialCommandGroup(
    Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition), elevatorSubsystem).withTimeout(.1),
    Commands.run(()->intakeSubsystem.intakeIn(), intakeSubsystem).until(()->intakeSubsystem.pivotAtSetpoint()),
    new ParallelCommandGroup(
    Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.intakePosition),elevatorSubsystem).withTimeout(.3),
    Commands.run(()->intakeSubsystem.stopRollerMotor(),intakeSubsystem).withTimeout(0.3)),
    new ParallelCommandGroup(
    Commands.run(()->intakeSubsystem.setRollerMotor(-IntakeConstants.rollerMotorSpeed),intakeSubsystem).withTimeout(0.5).andThen(Commands.run(()->intakeSubsystem.setRollerMotor(0),intakeSubsystem).withTimeout(0.1)),
    Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition),elevatorSubsystem)));
    //Commands.run(()->intakeSubsystem.intakeL1(),intakeSubsystem).until(()->intakeSubsystem.pivotAtSetpoint())).finallyDo(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition));

  private final Command intakeL1Command = new ParallelCommandGroup(
    Commands.run(()->intakeSubsystem.intakeL1(),intakeSubsystem),
    Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition), elevatorSubsystem));

  private final Command autoIntakeL1Command = new ParallelCommandGroup(
    Commands.run(()->intakeSubsystem.intakeL1(),intakeSubsystem).until(()->intakeSubsystem.pivotAtSetpoint()));

  private final Command autoIntakeInCommand = new SequentialCommandGroup(
  new ParallelCommandGroup(
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L4Position),elevatorSubsystem),
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition),armSubsystem),   
      Commands.run(()->intakeSubsystem.intakeIn(),intakeSubsystem)));


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
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
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

SwerveInputStream driveToLeftReefPost = SwerveInputStream.of(drivebase.getSwerveDrive(), ()->0.8*drivebase.getClosestReefPostLeftXDistance(), ()->0.8*drivebase.getClosestReefPostLeftYDistance());

SwerveInputStream driveToRightReefPost = SwerveInputStream.of(drivebase.getSwerveDrive(), ()->0.8*drivebase.getClosestReefPostRightXDistance(), ()->0.8*drivebase.getClosestReefPostRightYDistance());

SwerveInputStream driveToJPost = SwerveInputStream.of(drivebase.getSwerveDrive(), ()->0.8*drivebase.getJPostXDistance(), ()->0.8*drivebase.getJPostYDistance());

SwerveInputStream driveToEPost = SwerveInputStream.of(drivebase.getSwerveDrive(), ()->0.8*drivebase.getEPostXDistance(), ()->0.8*drivebase.getEPostYDistance());

SwerveInputStream autoTurnToReef = SwerveInputStream.of(drivebase.getSwerveDrive(),
() -> driverController.getLeftY() * -1,
() -> driverController.getLeftX() * -1)
.withControllerRotationAxis(()->drivebase.getClosestAprilTagRotationPID())
.deadband(OperatorConstants.DEADBAND)
.scaleTranslation(1)
.allianceRelativeControl(true);

SwerveInputStream autoTurnToFeederStation = SwerveInputStream.of(drivebase.getSwerveDrive(),
() -> driverController.getLeftY() * -1,
() -> driverController.getLeftX() * -1)
.withControllerRotationAxis(()->drivebase.getClosestFeederStationRotationPID())
.deadband(OperatorConstants.DEADBAND)
.scaleTranslation(1)
.allianceRelativeControl(true);
  
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

      NamedCommands.registerCommand("L3Height", 
      Commands.runOnce(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L3Position),elevatorSubsystem).withTimeout(2));

      NamedCommands.registerCommand("IntakeHeight", 
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.intakePosition),elevatorSubsystem).withTimeout(0.8));

      NamedCommands.registerCommand("StowHeight", 
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition),elevatorSubsystem).withTimeout(2));

      NamedCommands.registerCommand("PreScoreLeft", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.preScoreLeft),armSubsystem).withTimeout(1.5));
      
      NamedCommands.registerCommand("PreScoreLeftShort", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.preScoreLeft),armSubsystem).withTimeout(1));

      NamedCommands.registerCommand("PreScoreRight", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.preScoreRight),armSubsystem).withTimeout(1.5));

      NamedCommands.registerCommand("PreScoreRightShort", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.preScoreRight),armSubsystem).withTimeout(1));

      NamedCommands.registerCommand("VerticalPositionWithTimeout", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition),armSubsystem).withTimeout(.5));

      NamedCommands.registerCommand("VerticalPosition", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition),armSubsystem).withTimeout(1.5
      ));

      NamedCommands.registerCommand("descorePosition", 
      Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.descorePosition),armSubsystem).withTimeout(1.5));
      
      NamedCommands.registerCommand("driveToRightReefPost", drivebase.driveFieldOriented(driveToRightReefPost.withControllerRotationAxis(()-> 
      drivebase.getClosestAprilTagRotationPID())).until(()->drivebase.isInDistanceToleranceRight()));
    
      NamedCommands.registerCommand("driveToLeftReefPost", drivebase.driveFieldOriented(driveToLeftReefPost.withControllerRotationAxis(()-> 
      drivebase.getClosestAprilTagRotationPID())).until(()->drivebase.isInDistanceToleranceLeft()));

      NamedCommands.registerCommand("driveToJPost", drivebase.driveFieldOriented(driveToJPost.withControllerRotationAxis(()-> 
      drivebase.getClosestAprilTagRotationPID())).until(()->drivebase.isInDistanceToleranceRight()));
      
      NamedCommands.registerCommand("driveToEPost", drivebase.driveFieldOriented(driveToEPost.withControllerRotationAxis(()-> 
      drivebase.getClosestAprilTagRotationPID())).until(()->drivebase.isInDistanceToleranceLeft()));


      NamedCommands.registerCommand("climberOut", Commands.run(()->climberSubsystem.climberOut(), climberSubsystem).until(()->climberSubsystem.isClimberOut()));

      NamedCommands.registerCommand("stopArmMotor", Commands.run(()->armSubsystem.stopMotor(),armSubsystem).withTimeout(0.1));

      NamedCommands.registerCommand("IntakeIn", autoIntakeInCommand);

      NamedCommands.registerCommand("IntakeL1", autoIntakeL1Command);

      NamedCommands.registerCommand("climberOut", Commands.run(()->climberSubsystem.climberOut(), climberSubsystem));
      
      NamedCommands.registerCommand("grabCoral", new SequentialCommandGroup(
      new ParallelCommandGroup(
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.intakePosition),elevatorSubsystem).withTimeout(0.4),
      Commands.run(()->intakeSubsystem.stopRollerMotor(), intakeSubsystem).withTimeout(0.4)),

      new ParallelCommandGroup(
      Commands.run(()->intakeSubsystem.setRollerMotor(-IntakeConstants.rollerMotorSpeed),intakeSubsystem).withTimeout(0.5).andThen(Commands.run(()->intakeSubsystem.setRollerMotor(0),intakeSubsystem).withTimeout(0.1))),
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L4Position),elevatorSubsystem)));

      NamedCommands.registerCommand("runIntakeWithSensor", Commands.run(()->intakeSubsystem.runRollerWithSensor(),intakeSubsystem).until(()->intakeSubsystem.isCoralDetected()).andThen(Commands.run(()->intakeSubsystem.stopRollerMotor(),intakeSubsystem).withTimeout(0.001)));


      // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
   autoChooser = AutoBuilder.buildAutoChooser("W1C1");
    
  
   SmartDashboard.putData("Auto Chooser", autoChooser);
   SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
   SmartDashboard.putBoolean("Aligned To Reef", drivebase.isInDistanceToleranceEither());
   
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

    climberSubsystem.setDefaultCommand(Commands.run(()->{
      
      climberSubsystem.setMotor(0);
    }, climberSubsystem));
    
    armSubsystem.setDefaultCommand(Commands.run(()->{
      var leftCtrl = Math.min(MathUtil.applyDeadband(operatorController.getRawAxis(4),0.01), 0.0);
      var rotLeft = .4*Math.pow(leftCtrl, 2);
      var rightCtrl = Math.min(MathUtil.applyDeadband(operatorController.getRawAxis(3),0.01),0.0);
      var rotRight = .4*Math.pow(rightCtrl,2);
      SmartDashboard.putNumber("RotLeft", rotLeft);
      SmartDashboard.putNumber("LeftCtrl", leftCtrl);
      SmartDashboard.putNumber("RotRight", rotRight);
      //SmartDashboard.putNumber("Battery Voltage", )
     
      armSubsystem.setMotor(rotRight - rotLeft);
    }, armSubsystem));
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    intakeSubsystem.setDefaultCommand(Commands.run(()->intakeSubsystem.stopBothMotors(), intakeSubsystem));
                                

    if (Robot.isSimulation())
    {
     // driverController.square().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
    } 
    else {
      
       operatorController.triangle().onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.intakePosition),elevatorSubsystem));
       operatorController.circle().onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L4Position),elevatorSubsystem));
       operatorController.cross().onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition),elevatorSubsystem));
       operatorController.square().onTrue(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.L2Position),elevatorSubsystem));
       operatorController.button(15).whileTrue(Commands.run(()->elevatorSubsystem.resetEncoder(), elevatorSubsystem));
       operatorController.button(13).whileTrue(Commands.run(()->climberSubsystem.setMotor(ClimberConstants.climberMotorSpeed),climberSubsystem)).whileFalse(Commands.run(()->climberSubsystem.stopMotor(),climberSubsystem));
       operatorController.button(14).whileTrue(Commands.run(()->climberSubsystem.setMotor(-ClimberConstants.climberMotorSpeed),climberSubsystem)).whileFalse(Commands.run(()->climberSubsystem.stopMotor(),climberSubsystem));
       operatorController.L1().onTrue(Commands.run(()->climberSubsystem.setMotorPosition(ClimberConstants.climberOutPosition), climberSubsystem)).onFalse(climberSubsystem.getDefaultCommand());
       operatorController.R1().onTrue(Commands.run(()->climberSubsystem.setMotorPosition(ClimberConstants.climberInPosition), climberSubsystem)).onFalse(climberSubsystem.getDefaultCommand());
       operatorController.povDown().onTrue(Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition),armSubsystem)).onFalse(armSubsystem.getDefaultCommand());
      // operatorController.povLeft().onTrue(Commands.run(()->intakeSubsystem.setRollerMotor(-IntakeConstants.rollerMotorSpeed), intakeSubsystem)).whileFalse(Commands.run(()->intakeSubsystem.stopRollerMotor(),intakeSubsystem));
      // operatorController.povRight().onTrue(Commands.run(()->intakeSubsystem.setRollerMotor(IntakeConstants.rollerMotorSpeed), intakeSubsystem)).whileFalse(Commands.run(()->intakeSubsystem.stopRollerMotor(),intakeSubsystem));
      // operatorController.povUp().onTrue(Commands.run(()->intakeSubsystem.setPivotMotor(IntakeConstants.pivotMotorSpeed), intakeSubsystem)).whileFalse(Commands.run(()->intakeSubsystem.stopPivotMotor(),intakeSubsystem));
      //operatorController.povDown().onTrue(Commands.run(()->intakeSubsystem.setPivotMotor(-IntakeConstants.pivotMotorSpeed), intakeSubsystem)).whileFalse(Commands.run(()->intakeSubsystem.stopPivotMotor(),intakeSubsystem));
     operatorController.povLeft().onTrue(intakeInCommand);
     operatorController.povRight().onTrue(intakeOutCommand);
      operatorController.povUp().onTrue(intakeL1Command);
      operatorController.button(9).whileTrue(new ParallelCommandGroup(Commands.run(()->intakeSubsystem.setRollerMotor(-IntakeConstants.rollerMotorSpeed), intakeSubsystem),
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition), elevatorSubsystem)))
      .onFalse(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition), elevatorSubsystem));
      operatorController.button(10).whileTrue(new ParallelCommandGroup(Commands.run(()->intakeSubsystem.setRollerMotor(IntakeConstants.rollerMotorSpeed), intakeSubsystem),
      Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition), elevatorSubsystem)))
      .onFalse(Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition), elevatorSubsystem));

       driverController.R1().whileTrue(Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition),armSubsystem)).whileFalse(armSubsystem.getDefaultCommand());

       driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverController.R1().whileTrue(Commands.run(()->autoAlignToClosestAprilTag(),drivebase));
      // driverController.L1().whileTrue(Commands.run(()->autoAlignToClosestFeederStation(),drivebase));
      driverController.L1().onTrue(new ParallelCommandGroup(Commands.run(()->intakeSubsystem.setMotorPosition(IntakeConstants.algaePosition), intakeSubsystem),Commands.run(()->elevatorSubsystem.setMotorPosition(ElevatorConstants.stowPosition), elevatorSubsystem)));
       driverController.L2().whileTrue(drivebase.driveFieldOriented(driveToLeftReefPost.withControllerRotationAxis(()->drivebase.getClosestAprilTagRotationPID())));
       driverController.R2().whileTrue(drivebase.driveFieldOriented(driveToRightReefPost.withControllerRotationAxis(()->drivebase.getClosestAprilTagRotationPID())));
       driverController.circle().onTrue(Commands.run(()->climberSubsystem.climberOut(),climberSubsystem));
       driverController.triangle().onTrue(new SequentialCommandGroup(Commands.run(()->intakeSubsystem.setMotorPosition(IntakeConstants.intakeL1Position), intakeSubsystem).until(()->intakeSubsystem.pivotAtSetpoint()),Commands.run(()->intakeSubsystem.stopPivotMotor(), intakeSubsystem).withTimeout(0.1),
       new ParallelCommandGroup(Commands.run(()->elevatorSubsystem.setMotor(0), elevatorSubsystem),
       Commands.run(()->armSubsystem.setMotorPosition(ArmConstants.VerticalPosition+.5), armSubsystem))));
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
    System.out.println(drivebase.isRedAlliance());
    if(drivebase.isRedAlliance()){
      drivebase.driveFieldOriented(drivebase.getTargetSpeeds(driverController.getLeftY(), driverController.getLeftX(),
    drivebase.getClosestAprilTagRotation()));
    }
    else{
    drivebase.driveFieldOriented(drivebase.getTargetSpeeds(-driverController.getLeftY(), -driverController.getLeftX(),
    drivebase.getClosestAprilTagRotation()));
    }
  }

  private void autoAlignToClosestFeederStation(){
    if(drivebase.isRedAlliance()){
      drivebase.driveFieldOriented(drivebase.getTargetSpeeds(driverController.getLeftY(), driverController.getLeftX(),
    drivebase.getClosestFeederStationRotation()));
    }
    else{
    drivebase.driveFieldOriented(drivebase.getTargetSpeeds(-driverController.getLeftY(), -driverController.getLeftX(),
    drivebase.getClosestFeederStationRotation()));
    }

}
}
