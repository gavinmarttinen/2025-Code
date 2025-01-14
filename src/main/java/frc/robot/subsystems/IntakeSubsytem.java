package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsytem extends SubsystemBase{
    private final SparkMax intakeMotor = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax intakePivotMotor = new SparkMax(0, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final PIDController pidController = new PIDController(0, 0, 0);
    private final DigitalInput sensor = new DigitalInput(0);

public IntakeSubsytem() {
}

@Override
public void periodic() {
}

public void setMotorPosition(double setpoint) {
intakePivotMotor.set(pidController.calculate(encoder.get(), setpoint));
}

public void runIntakeWithSensor() {
    if(sensor.get()){
        intakeMotor.set(0);
    }
}

public void runIntake() {
    intakeMotor.set(.5);
}

public void runOuttake() {
    intakeMotor.set(-.5);
}
}
