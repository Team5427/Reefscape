package team5427.frc.robot.subsystems.SupremeEffector.io;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants.SupremeEffectorConstants;
import team5427.frc.robot.subsystems.SupremeEffector.io.SupremeIO.SupremeIOInputs;
import team5427.lib.motors.real.SteelTalonFX;

public class SupremeIOTalon implements SupremeIO {
  private SteelTalonFX pivotMotor;
  private SteelTalonFX coralMotor;
  private SteelTalonFX algaeMotor;

  private CANrange canRange;

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

  private StatusSignal<Distance> canRangeDistance;
  private StatusSignal<Boolean> canRangeDetectedObject;

  public SupremeIOTalon() {
    canRange = new CANrange(SupremeEffectorConstants.kCanRangeId.getDeviceNumber());
    CANrangeConfiguration canrangeConfiguration = new CANrangeConfiguration();
    canrangeConfiguration.FovParams.FOVRangeX = 5.0;
    canrangeConfiguration.FovParams.FOVCenterY = 5.0;
    canrangeConfiguration.ProximityParams.ProximityThreshold =
        SupremeEffectorConstants.kCanRangeDetectionDistance.in(Meters);
    canrangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    canRange.getConfigurator().apply(canrangeConfiguration);

    canRangeDistance = canRange.getDistance();
    canRangeDetectedObject = canRange.getIsDetected();

    pivotMotor = new SteelTalonFX(SupremeEffectorConstants.kPivotMotorId);
    pivotMotor.apply(SupremeEffectorConstants.kPivotMotorConfiguration);
    pivotMotor.useTorqueCurrentFOC(true);
    pivotMotorAngularAcceleration = pivotMotor.getTalonFX().getAcceleration();
    pivotMotorAngularVelocity = pivotMotor.getTalonFX().getVelocity();
    pivotMotorCurrent = pivotMotor.getTalonFX().getStatorCurrent();
    pivotMotorPosition = pivotMotor.getTalonFX().getPosition();
    pivotMotorVoltage = pivotMotor.getTalonFX().getMotorVoltage();

    coralMotor = new SteelTalonFX(SupremeEffectorConstants.kCoralMotorId);
    coralMotor.apply(SupremeEffectorConstants.kCoralMotorConfiguration);
    coralMotor.useTorqueCurrentFOC(true);
    coralMotorAngularAcceleration = coralMotor.getTalonFX().getAcceleration();
    coralMotorAngularVelocity = coralMotor.getTalonFX().getVelocity();
    coralMotorCurrent = coralMotor.getTalonFX().getStatorCurrent();
    coralMotorPosition = coralMotor.getTalonFX().getPosition();
    coralMotorVoltage = coralMotor.getTalonFX().getMotorVoltage();

    algaeMotor = new SteelTalonFX(SupremeEffectorConstants.kAlgaeMotorId);
    algaeMotor.apply(SupremeEffectorConstants.kAlgaeMotorConfiguration);
    algaeMotor.useTorqueCurrentFOC(true);
    algaeMotorAngularAcceleration = algaeMotor.getTalonFX().getAcceleration();
    algaeMotorAngularVelocity = algaeMotor.getTalonFX().getVelocity();
    algaeMotorCurrent = algaeMotor.getTalonFX().getStatorCurrent();
    algaeMotorPosition = algaeMotor.getTalonFX().getPosition();
    algaeMotorVoltage = algaeMotor.getTalonFX().getMotorVoltage();

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
        algaeMotorVoltage,
        canRangeDistance,
        canRangeDetectedObject);

    pivotMotor.setEncoderPosition(SupremeEffectorConstants.kZeroPosition);
  }

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
        algaeMotorVoltage,
        canRangeDistance,
        canRangeDetectedObject);

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

    inputs.canRangeConnected = canRange.isConnected();
    inputs.canRangeDetectedObject = canRangeDetectedObject.getValue();
    inputs.canRangeDistance = canRangeDistance.getValue();
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
