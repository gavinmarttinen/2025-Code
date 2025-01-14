package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX elevatorMotor = new TalonFX(0);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final PIDController pidController = new PIDController(0, 0, 0);

public ElevatorSubsystem() {
}

@Override
public void periodic() {
}

public void setMotorPosition(double setpoint) {
elevatorMotor.set(pidController.calculate(encoder.get(), setpoint));
}


}
