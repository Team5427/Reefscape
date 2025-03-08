package team5427.frc.robot.subsystems.Climb.io;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants.ClimbConstants;
import team5427.lib.motors.real.SteelTalonFX;

public class ClimbIOTalon implements ClimbIO {

  private SteelTalonFX hookServo;

  private StatusSignal<Angle> hookPosition;
  private StatusSignal<AngularVelocity> hookVelocity;
  private StatusSignal<AngularAcceleration> hookAcceleration;
  private StatusSignal<Current> hookServoCurrent;
  private StatusSignal<Voltage> hookServoVoltage;

  public ClimbIOTalon() {
    hookServo = new SteelTalonFX(ClimbConstants.kHookServoId);
    hookServo.apply(ClimbConstants.kServoConfiguration);

    hookPosition = hookServo.getTalonFX().getPosition();
    hookVelocity = hookServo.getTalonFX().getVelocity();
    hookAcceleration = hookServo.getTalonFX().getAcceleration();

    hookServoCurrent = hookServo.getTalonFX().getStatorCurrent();
    hookServoVoltage = hookServo.getTalonFX().getMotorVoltage();

    hookServo.setEncoderPosition(Rotation2d.kZero);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, hookPosition, hookVelocity, hookAcceleration, hookServoCurrent, hookServoVoltage);

    ParentDevice.optimizeBusUtilizationForAll(hookServo.getTalonFX());
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // TODO Auto-generated method stub

    BaseStatusSignal.refreshAll(
        hookPosition, hookVelocity, hookAcceleration, hookServoCurrent, hookServoVoltage);

    inputs.hookPosition = Rotation2d.fromRotations(hookPosition.getValue().in(Rotations));
    inputs.hookVelocity = hookVelocity.getValue();
    inputs.hookAcceleration = hookAcceleration.getValue();

    inputs.hookServoCurrent = hookServoCurrent.getValue();
    inputs.hookServoVoltage = hookServoVoltage.getValue();
  }

  @Override
  public void setHookSetpoint(Rotation2d setpoint) {
    hookServo.setSetpoint(setpoint);
  }

  @Override
  public void setHookVoltage(Voltage volts) {
   hookServo.setRawVoltage(volts.in(Volt));
  }

  @Override
  public void setHookPosition(Rotation2d angle){
    hookServo.setEncoderPosition(angle);
  }
}
