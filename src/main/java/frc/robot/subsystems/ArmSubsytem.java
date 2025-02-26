package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ArmSubsytem extends SubsystemBase{
    private final TalonFX armMotor = new TalonFX(ArmConstants.armMotorID);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(9);
    private final PIDController pidController = new PIDController(1.2, 0.005, 0);
   //private final TalonFXConfigurator config = armMotor.getConfigurator();

public ArmSubsytem() {
    // System.out.println(encoder.get());
    pidController.enableContinuousInput(0, 1);
   armMotor.setInverted(false);
    
}

@Override
public void periodic() {
    System.out.println("arm Encoder " + encoder.get());
    SmartDashboard.putNumber("armEncoder",encoder.get());
}

public void setMotorPosition(double setpoint) {
armMotor.set(pidController.calculate(encoder.get(), setpoint));
}

public void setMotor(double speed) {
    armMotor.set(speed);
}

}
