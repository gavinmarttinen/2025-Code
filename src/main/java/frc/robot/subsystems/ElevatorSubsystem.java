package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    private final TalonFX elevatorMotor1 = new TalonFX(ElevatorConstants.elevatorMotor1ID);
   // private final DutyCycleEncoder encoder = new DutyCycleEncoder(ElevatorConstants.elevatorEncoderID);
    private final StatusSignal encoder = elevatorMotor.getPosition();
    private final PIDController pidController = new PIDController(0, 0, 0);
    private final DigitalInput coralSensor = new DigitalInput(ElevatorConstants.sensorID);
   // private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchChannel);
private final VoltageOut m_voltReq = new VoltageOut(0.0);
private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.3, 0.1);

public ElevatorSubsystem() {
   elevatorMotor1.setControl(new Follower(elevatorMotor.getDeviceID(), false));
   
//elevatorMotor1.setControl(new Follower(15, false));
var talonFXConfigs = new TalonFXConfiguration();
// in init function, set slot 0 gains
var slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = 0.04087;
slot0Configs.kV = 0.12261;
slot0Configs.kA = 0.0033489;
slot0Configs.kP = ElevatorConstants.kP; // An error of 1 rotation results in 2.4 V output
slot0Configs.kI = ElevatorConstants.kI; // no output for integrated error
slot0Configs.kD = ElevatorConstants.kD; // A velocity of 1 rps results in 0.1 V output
slot0Configs.kG = ElevatorConstants.kG;

// set Motion Magic Settings
var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 80;
motionMagicConfigs.MotionMagicAcceleration = 80;
motionMagicConfigs.MotionMagicJerk = 1600;
//elevatorMotor.setInverted(true);
elevatorMotor.getConfigurator().apply(talonFXConfigs);
}

@Override
public void periodic() {
   // System.out.println(elevatorMotor.getPosition());
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
final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
// if (limitSwitch.get()) {
//     elevatorMotor.setPosition(0);
// }

elevatorMotor.setControl(m_request.withPosition(setpoint));

}

private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> elevatorMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.dynamic(direction);
}

}
