package team5427.frc.robot.subsystems.SupremeEffector.io;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.subsystems.SupremeEffector.io.SupremeIO.SupremeIOInputs;
import team5427.lib.motors.real.SteelTalonFX;

public class SupremeIOTalon implements SupremeIO {
  private SteelTalonFX pivotMotor;
  private SteelTalonFX coralMotor;
  private SteelTalonFX algaeMotor;

  private StatusSignal<Angle> pivotMotorPosition;
  private StatusSignal<Angle> coralMotorPosition;
  private StatusSignal<Angle> algaeMotorPosition;

  private StatusSignal<AngularVelocity> pivotMotorAngularVelocity;
  private StatusSignal<AngularVelocity> coralMotorAngularVelocity;
  private StatusSignal<AngularVelocity> algaeMotorAngularVelocity;

  private StatusSignal<AngularAcceleration> pivotMotorAngularAcceleration;
  private StatusSignal<AngularAcceleration> coralMotorAngularAcceleration;
  private StatusSignal<AngularAcceleration> algaeMotorAngularAcceleration;

  private StatusSignal<Current> pivotMotorCurrent;
  private StatusSignal<Current> coralMotorCurrent;
  private StatusSignal<Current> algaeMotorCurrent;

  private StatusSignal<Voltage> pivotMotorVoltage;
  private StatusSignal<Voltage> coralMotorVoltage;
  private StatusSignal<Voltage> algaeMotorVoltage;

  public SupremeIOTalon() {}

  @Override
  public void updateInputs(SupremeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotMotorPosition,
        pivotMotorAngularAcceleration,
        pivotMotorAngularVelocity,
        pivotMotorCurrent,
        pivotMotorVoltage,
        coralMotorAngularAcceleration,
        coralMotorAngularVelocity,
        coralMotorCurrent,
        coralMotorPosition,
        coralMotorVoltage,
        algaeMotorAngularAcceleration,
        algaeMotorAngularVelocity,
        algaeMotorCurrent,
        algaeMotorPosition,
        algaeMotorVoltage);

    inputs.algaeMotorConnected = algaeMotor.getTalonFX().isConnected();
    inputs.algaeMotorLinearAcceleration =
        MetersPerSecondPerSecond.of(
            algaeMotor.getEncoderAcceleration(algaeMotorAngularAcceleration));
    inputs.algaeMotorLinearVelocity =
        MetersPerSecond.of(algaeMotor.getEncoderVelocity(algaeMotorAngularVelocity));
    inputs.algaeMotorPosition = Meters.of(algaeMotor.getEncoderPosition(algaeMotorPosition));
    inputs.algaeMotorStatorCurrent = algaeMotorCurrent.getValue();
    inputs.algaeMotorVoltage = algaeMotorVoltage.getValue();

    inputs.coralMotorConnected = coralMotor.getTalonFX().isConnected();
    inputs.coralMotorLinearAcceleration =
        MetersPerSecondPerSecond.of(
            coralMotor.getEncoderAcceleration(coralMotorAngularAcceleration));
    inputs.coralMotorLinearVelocity =
        MetersPerSecond.of(coralMotor.getEncoderVelocity(coralMotorAngularVelocity));
    inputs.coralMotorPosition = Meters.of(coralMotor.getEncoderPosition(coralMotorPosition));
    inputs.coralMotorStatorCurrent = coralMotorCurrent.getValue();
    inputs.coralMotorVoltage = coralMotorVoltage.getValue();

    inputs.pivotMotorConnected = pivotMotor.getTalonFX().isConnected();
    inputs.pivotMotorAngularAcceleration =
        RotationsPerSecondPerSecond.of(
            pivotMotor.getEncoderAcceleration(pivotMotorAngularAcceleration));
    inputs.pivotMotorAngularVelocity =
        RotationsPerSecond.of(pivotMotor.getEncoderVelocity(pivotMotorAngularVelocity));
    inputs.pivotMotorPosition =
        Rotation2d.fromRotations(pivotMotor.getEncoderPosition(pivotMotorPosition));
    inputs.pivotMotorStatorCurrent = pivotMotorCurrent.getValue();
    inputs.pivotMotorVoltage = pivotMotorVoltage.getValue();
  }

  @Override
  public void setPivotMotorSetpoint(Rotation2d angle) {
    pivotMotor.setSetpoint(angle);
  }

  @Override
  public void setCoralMotorSetpoint(LinearVelocity velocity) {
    coralMotor.setSetpoint(velocity.in(MetersPerSecond));
  }

  @Override
  public void setAlgaeMotorSetpoint(LinearVelocity velocity) {
    algaeMotor.setSetpoint(velocity.in(MetersPerSecond));
  }

  @Override
  public void setPivotMotorPosition(Rotation2d position) {
    pivotMotor.setEncoderPosition(position);
  }
}
