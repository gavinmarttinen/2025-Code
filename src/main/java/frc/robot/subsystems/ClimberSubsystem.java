package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    private final TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotorID);
   private final DutyCycleEncoder encoder = new DutyCycleEncoder(7);
   private final PIDController pidController = new PIDController(ClimberConstants.P, ClimberConstants.I, ClimberConstants.D);

public ClimberSubsystem() {
    
   climberMotor.setInverted(false);
    
}

@Override
 public void periodic() {
    System.out.println("Climb Encoder"+encoder.get());
 }

 public void setMotorPosition(double setpoint) {
 climberMotor.set(pidController.calculate(encoder.get(), setpoint));
 }

 public void setMotor(double speed) {
     climberMotor.set(speed);
 }
public void stopMotor() {
     climberMotor.set(0);
 }
}
