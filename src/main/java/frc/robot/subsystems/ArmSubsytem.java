package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsytem extends SubsystemBase{
    private final TalonFX armMotor = new TalonFX(ArmConstants.armMotorID);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.armEncoderID);
    private final PIDController pidController = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);

public ArmSubsytem() {
    pidController.enableContinuousInput(0, 1);
   armMotor.setInverted(false);
    
}

@Override
public void periodic() {
    SmartDashboard.putNumber("armEncoder",encoder.get());
}

public void setMotorPosition(double setpoint) {
armMotor.set(pidController.calculate(encoder.get(), setpoint));
}

public void setMotor(double speed) {
    armMotor.set(speed);
}
public void stopMotor(){
    armMotor.set(0);
}

}
