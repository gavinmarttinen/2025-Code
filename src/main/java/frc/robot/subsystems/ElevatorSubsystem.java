package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
private final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
private final TalonFX elevatorMotor1 = new TalonFX(ElevatorConstants.elevatorMotor1ID);
private final VoltageOut m_voltReq = new VoltageOut(0.0);

public ElevatorSubsystem() {
   elevatorMotor1.setControl(new Follower(elevatorMotor.getDeviceID(), false));
   
var talonFXConfigs = new TalonFXConfiguration();
// in init function, set slot 0 gains
var slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = ElevatorConstants.kS;
slot0Configs.kV = ElevatorConstants.kV;
slot0Configs.kA = ElevatorConstants.kA;
slot0Configs.kP = ElevatorConstants.kP; // An error of 1 rotation results in 2.4 V output
slot0Configs.kI = ElevatorConstants.kI; // no output for integrated error
slot0Configs.kD = ElevatorConstants.kD; // A velocity of 1 rps results in 0.1 V output
slot0Configs.kG = ElevatorConstants.kG;

// set Motion Magic Settings
var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.CruiseVelocity;
motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.Acceleration;
motionMagicConfigs.MotionMagicJerk = ElevatorConstants.Jerk;
elevatorMotor.getConfigurator().apply(talonFXConfigs);
}

@Override
public void periodic() {
    SmartDashboard.putNumber("elevatorMotorEncoder",elevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("elevator motor voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("elevator Motor 1 voltage", elevatorMotor1.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("elevator motor current", elevatorMotor.getSupplyCurrent().getValueAsDouble());
}
public void setMotor(double speed) {
    elevatorMotor.set(speed);
}


public void setMotorPosition(double setpoint) {
// create a position closed-loop request, voltage output, slot 0 configs
final MotionMagicVoltage m_request = new MotionMagicVoltage(0);


elevatorMotor.setControl(m_request.withPosition(setpoint));

}

public void resetEncoder(){
   elevatorMotor.set(-0.1);
   if(elevatorMotor.getSupplyCurrent().getValueAsDouble()>2){
      elevatorMotor.setPosition(0);
   }
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