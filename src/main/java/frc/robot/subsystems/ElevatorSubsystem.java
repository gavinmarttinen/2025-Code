package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    private final TalonFX elevatorMotor1 = new TalonFX(ElevatorConstants.elevatorMotor1ID);
   // private final DutyCycleEncoder encoder = new DutyCycleEncoder(ElevatorConstants.elevatorEncoderID);
    private final StatusSignal encoder = elevatorMotor.getPosition();
    private final PIDController pidController = new PIDController(0, 0, 0);
    private final DigitalInput coralSensor = new DigitalInput(ElevatorConstants.sensorID);
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchChannel);

public ElevatorSubsystem() {
   elevatorMotor1.setControl(new Follower(elevatorMotor.getDeviceID(), false));
//elevatorMotor1.setControl(new Follower(15, false));

// in init function, set slot 0 gains
var slot0Configs = new Slot0Configs();
slot0Configs.kP = ElevatorConstants.kP; // An error of 1 rotation results in 2.4 V output
slot0Configs.kI = ElevatorConstants.kI; // no output for integrated error
slot0Configs.kD = ElevatorConstants.kD; // A velocity of 1 rps results in 0.1 V output
slot0Configs.kG = ElevatorConstants.kG;
elevatorMotor.getConfigurator().apply(slot0Configs);
}

@Override
public void periodic() {
    System.out.println(elevatorMotor.getPosition());
    SmartDashboard.putNumber("elevatorMotorEncoder",elevatorMotor.getPosition().getValueAsDouble());
}
public void setMotor(double speed) {
    elevatorMotor.set(speed);
}

public void intakeCoral(){
if(coralSensor.get()){
setMotor(ElevatorConstants.intakePosition);}

}


public void setMotorPosition(double setpoint) {
// create a position closed-loop request, voltage output, slot 0 configs
final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
if (limitSwitch.get()) {
    elevatorMotor.setPosition(0);
}
// set position to 10 rotations
elevatorMotor.setControl(m_request.withPosition(setpoint));
}


}
