package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
   private final SparkMax rollerMotor = new SparkMax(IntakeConstants.rollerMotorID, MotorType.kBrushless);
   private final TalonFX pivotMotor = new TalonFX(IntakeConstants.pivotMotorID);
   private final DutyCycleEncoder encoder = new DutyCycleEncoder(8);
   private final PIDController PIDController = new PIDController(IntakeConstants.P, IntakeConstants.I, IntakeConstants.D);

public IntakeSubsystem() {
    
   rollerMotor.setInverted(false);
   pivotMotor.setInverted(false);
   PIDController.setTolerance(0.01);
   
    
}

@Override
 public void periodic() {
    SmartDashboard.putNumber("Intake Encoder", encoder.get());
    SmartDashboard.putNumber("Roller Motor Current", rollerMotor.getOutputCurrent());
 }

 public void setMotorPosition(double setpoint) {
 pivotMotor.set(PIDController.calculate(encoder.get(), setpoint));
 }

 public void setRollerMotor(double speed) {
     rollerMotor.set(speed);
 }

 public void setPivotMotor(double speed) {
   pivotMotor.set(speed);
}

public void stopPivotMotor() {
     pivotMotor.set(0);
 }

 public void stopRollerMotor() {
   rollerMotor.set(0);
}

public void stopBothMotors(){
   rollerMotor.set(0.1);
   pivotMotor.set(0);
}

public void deployIntake(){
   pivotMotor.set(PIDController.calculate(encoder.get(), IntakeConstants.intakeOutPosition));
   setRollerMotor(IntakeConstants.rollerMotorSpeed);
   if(rollerMotor.getOutputCurrent()>IntakeConstants.outputCurrent){
      setRollerMotor(0.1);
   }
}

public void runRollerWithSensor(){
   setRollerMotor(IntakeConstants.rollerMotorSpeed);
   if(rollerMotor.getOutputCurrent()>IntakeConstants.outputCurrent){
      stopRollerMotor();
   }
}

public void intakeIn(){
   pivotMotor.set(PIDController.calculate(encoder.get(), IntakeConstants.intakeInPosition));
}

public void intakeL1(){
   pivotMotor.set(PIDController.calculate(encoder.get(), IntakeConstants.intakeL1Position));
}

public boolean pivotAtSetpoint(){
   return PIDController.atSetpoint();
}

public boolean isCoralDetected(){
   if (rollerMotor.getOutputCurrent()>IntakeConstants.outputCurrent){
      return true;
   }
   else{
      return false;
   }
}

   public boolean isCoralDetectedAndPivotAtSetpoint(){
      if (rollerMotor.getOutputCurrent()>IntakeConstants.outputCurrent&&PIDController.atSetpoint()){
         return true;
      }
      else{
         return false;
      }
}
}