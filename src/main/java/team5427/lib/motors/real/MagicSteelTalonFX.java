package team5427.lib.motors.real;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.IMotorController;
import team5427.lib.motors.real.MotorConfiguration.MotorMode;

public class MagicSteelTalonFX implements IMotorController {

  private CANDeviceId id;
  private TalonFX talonFX;
  private MotorConfiguration configuration;

  private double setpoint;

  private boolean withFOC;

  public TalonFXConfiguration talonConfig;

  public MagicSteelTalonFX(CANDeviceId id) {
    this.id = id;

    talonFX = new TalonFX(this.id.getDeviceNumber());

    withFOC = false;
  }

  @Override
  public void apply(MotorConfiguration configuration) {

    talonConfig = new TalonFXConfiguration();

    talonConfig.Feedback.SensorToMechanismRatio =
        configuration.gearRatio.getSensorToMechanismRatio();

    talonConfig.Slot0.kP = configuration.kP;
    talonConfig.Slot0.kI = configuration.kI;
    talonConfig.Slot0.kD = configuration.kD;
    talonConfig.Slot0.kS = configuration.kS;
    talonConfig.Slot0.kV = configuration.kV;
    talonConfig.Slot0.kA = configuration.kA;
    talonConfig.Slot0.kG = configuration.kG;
    talonConfig.Slot0.GravityType =
        configuration.isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;

    withFOC = configuration.withFOC;

    switch (configuration.mode) {
      case kFlywheel:
        talonConfig.ClosedLoopGeneral.ContinuousWrap = false;
      case kServo:
      case kLinear:
        talonConfig.ClosedLoopGeneral.ContinuousWrap = true;
        break;
      default:
        break;
    }

    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.CurrentLimits.StatorCurrentLimit = configuration.currentLimit;
    talonConfig.CurrentLimits.SupplyCurrentLimit = configuration.currentLimit;

    talonConfig.MotionMagic.MotionMagicAcceleration = configuration.altA;
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = configuration.altV;
    talonConfig.MotionMagic.MotionMagicJerk = configuration.altJ;

    talonFX.getConfigurator().apply(talonConfig);
  }

  /**
   * @param setpoint - Rotation2d Flywheel: Rotation2d per Second Servo: Rotation2d Linear:
   *     Rotation2d
   */
  @Override
  public void setSetpoint(Rotation2d setpoint) {
    switch (configuration.mode) {
      case kFlywheel:
        talonFX.setControl(
            new MotionMagicVelocityVoltage(setpoint.getRotations())
                .withEnableFOC(withFOC)
                .withFeedForward(configuration.kFF));
        break;
      case kServo:
      case kLinear:
        talonFX.setControl(
            new MotionMagicVoltage(setpoint.getMeasure())
                .withEnableFOC(withFOC)
                .withFeedForward(configuration.kFF));
        break;
      default:
        talonFX.setControl(
            new MotionMagicVoltage(setpoint.getMeasure())
                .withEnableFOC(withFOC)
                .withFeedForward(configuration.kFF));

        break;
    }
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }

  @Override
  public void setEncoderPosition(double position) {
    talonFX.setPosition(position);
  }

  @Override
  public void setEncoderPosition(Rotation2d position) {
    talonFX.setPosition(position.getRotations());
  }

  /**
   * @return rotations if a servo, or meters if a flywheel or linear
   */
  @Override
  public double getEncoderPosition() {
    if (configuration.mode != MotorMode.kServo) {
      // converts to meters
      return talonFX.getPosition().getValue().in(Rotation)
          * Math.PI
          * configuration.finalDiameterMeters;
    }
    return talonFX.getPosition().getValueAsDouble();
  }

  /**
   * @return rotations per minute if a servo, meters per second if a linear or flywheel
   */
  @Override
  public double getEncoderVelocity() {
    if (configuration.mode != MotorMode.kServo) {
      // converts to meters
      return talonFX.getVelocity().getValue().in(RotationsPerSecond)
          * Math.PI
          * configuration.finalDiameterMeters;
    }
    // converts to RPM
    return talonFX.getVelocity().getValueAsDouble() * 60.0;
  }

  @Override
  public void setRawPercentage(double percentage) {
    talonFX.set(percentage);
  }

  @Override
  public void setRelativePercentage(double percentage) {
    talonFX.setVoltage(percentage * talonFX.getSupplyVoltage().getValueAsDouble());
  }

  @Override
  public void setRawVoltage(double voltage) {
    talonFX.setVoltage(voltage);
  }

  @Override
  public double getError() {
    return talonFX.getClosedLoopError().getValueAsDouble();
  }

  public void setFOC(boolean foc) {
    withFOC = foc;
  }

  public TalonFX getTalonFX() {
    return talonFX;
  }

  /**
   * @param setpoint Flywheel - m/s Linear - meters Servo - Rotations
   */
  @Override
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
    switch (configuration.mode) {
      case kFlywheel:
        talonFX.setControl(
            new MotionMagicVelocityVoltage(setpoint)
                .withEnableFOC(withFOC)
                .withFeedForward(configuration.kFF));
        break;
      case kServo:
        talonFX.setControl(
            new MotionMagicVoltage(setpoint)
                .withEnableFOC(withFOC)
                .withFeedForward(configuration.kFF));
        DriverStation.reportWarning(
            "Warning: TalonFX motor with the id "
                + talonFX.getDeviceID()
                + " is a Servo set with a double setpoint./n Use Rotation2d instead.",
            false);
      case kLinear:
        setpoint = setpoint / (Math.PI * configuration.finalDiameterMeters);
        talonFX.setControl(
            new MotionMagicVoltage(setpoint)
                .withEnableFOC(withFOC)
                .withFeedForward(configuration.kFF));
        break;
      default:
        talonFX.setControl(
            new MotionMagicVoltage(setpoint)
                .withEnableFOC(withFOC)
                .withFeedForward(configuration.kFF));
        break;
    }
  }
}
