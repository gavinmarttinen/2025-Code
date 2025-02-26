// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.Constants.ElevatorConstants;

// public class ClimberSubsystem extends SubsystemBase{
//     private final TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotorID);
//    // private final DutyCycleEncoder encoder = new DutyCycleEncoder(30);
//     private final PIDController pidController = new PIDController(1.2, 0.005, 0);
//    //private final TalonFXConfigurator config = armMotor.getConfigurator();

// public ClimberSubsystem() {
//     // System.out.println(encoder.get());
    
//    climberMotor.setInverted(false);
    
// }

// @Override
// // public void periodic() {
// //     System.out.println("arm Encoder " + encoder.get());
// //     SmartDashboard.putNumber("armEncoder",encoder.get());
// // }

// //  public void setMotorPosition(double setpoint) {
// // climberMotor.set(pidController.calculate(encoder.get(), setpoint));
// //  }

// // public void setMotor(double speed) {
// //     climberMotor.set(speed);
// // }
// // public void stopMotor() {
// //     climberMotor.set(0);
// // }
// }
