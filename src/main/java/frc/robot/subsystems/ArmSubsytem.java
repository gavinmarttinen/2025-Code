package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsytem extends SubsystemBase{
    private final SparkMax armMotor = new SparkMax(15, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(1);
    private final PIDController pidController = new PIDController(0, 0, 0);

public ArmSubsytem() {
}

@Override
public void periodic() {
}

public void setMotorPosition(double setpoint) {
armMotor.set(pidController.calculate(encoder.get(), setpoint));
}

}
