package team5427.lib.motors.real;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.IMotorController;
import team5427.lib.motors.real.MotorConfiguration.MotorMode;

public class MagicSteelTalonFX implements IMotorController {

  private CANDeviceId id;
  private TalonFX talonFX;
  private MotorConfiguration configuration;

  // private double positionConversionFactor;
  // // This is Radians/Second
  // private double velocityConversionFactor;

  private double setpoint;

  private boolean withFOC;

  public TalonFXConfiguration talonConfig;

  public TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(Amps.of(0.0));
  public MotionMagicTorqueCurrentFOC positionTorqueCurrentFOCRequest =
      new MotionMagicTorqueCurrentFOC(Rotation.of(0.0));
  public MotionMagicVelocityVoltage velocityVoltageRequest =
      new MotionMagicVelocityVoltage(RotationsPerSecond.of(0.0));
  public MotionMagicDutyCycle positionDutyCycleRequest = new MotionMagicDutyCycle(Rotation.of(0.0));
  public MotionMagicVelocityDutyCycle velocityDutyCycleRequest =
      new MotionMagicVelocityDutyCycle(RotationsPerSecond.of(0.0));
  public MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentFOCRequest =
      new MotionMagicVelocityTorqueCurrentFOC(RotationsPerSecond.of(0.0));
  private boolean useTorqueCurrentFOC = false;

  public MagicSteelTalonFX(CANDeviceId id) {
    this.id = id;

    talonFX = new TalonFX(this.id.getDeviceNumber(), this.id.getBus());

    withFOC = false;

    // position = talonFX.getPosition();
    // velocity = talonFX.getVelocity();
  }

  @Override
  public void apply(MotorConfiguration configuration) {
    this.configuration = configuration;
    talonConfig = new TalonFXConfiguration();

    // positionConversionFactor = configuration.unitConversionRatio;
    // velocityConversionFactor = configuration.unitConversionRatio / 60.0;

    talonConfig.Feedback.SensorToMechanismRatio =
        configuration.gearRatio.getSensorToMechanismRatio();

    switch (configuration.idleState) {
      case kBrake:
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        break;
      case kCoast:
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        break;
      default:
        break;
    }
    talonConfig.MotorOutput.Inverted =
        configuration.isInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

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
    talonConfig.FutureProofConfigs = true;

    switch (configuration.mode) {
      case kFlywheel:
        talonConfig.ClosedLoopGeneral.ContinuousWrap = false;
        break;
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
    talonConfig.CurrentLimits.SupplyCurrentLimit = configuration.currentLimit * 0.5;

    talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = configuration.currentLimit;
    talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -configuration.currentLimit;

    talonConfig.MotionMagic.MotionMagicAcceleration = configuration.altA;
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = configuration.altV;
    talonConfig.MotionMagic.MotionMagicJerk = configuration.altJ;

    talonFX.getConfigurator().apply(talonConfig);
  }

  // /**
  // * @param setpoint - Rotation2d
  // **/
  // @Override
  // public void setSetpoint(Rotation2d setpoint) {
  // switch (configuration.mode) {
  // case kFlywheel:
  // talonFX.setControl(new
  // VelocityVoltage(setpoint.getRotations()).withEnableFOC(withFOC));
  // break;
  // case kServo:
  // case kLinear:
  // talonFX.setControl(new
  // PositionVoltage(setpoint.getRotations()).withEnableFOC(withFOC));
  // break;
  // default:
  // talonFX.setControl(new
  // VelocityVoltage(setpoint.getRotations()).withEnableFOC(withFOC));
  // break;
  // }
  // }
  /**
   * @param setpoint - Rotation2d Flywheel: Rotation2d per Second Servo: Rotation2d Linear:
   *     Rotation2d
   */
  @Override
  public void setSetpoint(Rotation2d setpoint) {
    switch (configuration.mode) {
      case kFlywheel:
        this.setpoint = setpoint.getRotations();
        if (isUsingTorqueCurrentFOC()) {
          talonFX.setControl(
              velocityTorqueCurrentFOCRequest.withVelocity(
                  RotationsPerSecond.of(setpoint.getRotations())));
        } else {
          talonFX.setControl(
              velocityVoltageRequest
                  .withVelocity(RotationsPerSecond.of(setpoint.getRotations()))
                  .withEnableFOC(withFOC));
        }
        break;
      case kServo:
      case kLinear:
        this.setpoint = setpoint.getRotations();
        // Logger.recordOutput("Setpoint Of Steer" + talonFX.getDeviceID(), setpoint.getDegrees());
        if (isUsingTorqueCurrentFOC()) {
          talonFX.setControl(
              positionTorqueCurrentFOCRequest.withPosition(Rotation.of(setpoint.getRotations())));
        } else {

          talonFX.setControl(
              positionDutyCycleRequest
                  .withPosition(Rotation.of(setpoint.getRotations()))
                  .withEnableFOC(withFOC));
        }
        break;
      default:
        this.setpoint = setpoint.getRotations();
        talonFX.setControl(
            velocityVoltageRequest
                .withVelocity(RotationsPerSecond.of(setpoint.getRotations()))
                .withEnableFOC(withFOC));
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

    // if (configuration.mode != MotorMode.kServo) {
    // // converts to meters
    // return position.getValue().in(Rotation) * Math.PI *
    // configuration.finalDiameterMeters;
    // }
    // return position.getValueAsDouble();
    return 0;
  }

  /**
   * @return rotations if a servo, or meters if a flywheel or linear
   */
  public double getEncoderPosition(StatusSignal<Angle> position) {

    if (configuration.mode != MotorMode.kServo) {
      // converts to meters
      return position.getValue().in(Rotation) * Math.PI * configuration.finalDiameterMeters;
    }
    return position.getValueAsDouble();
  }

  // public void updateStatusSignals() {
  // BaseStatusSignal.refreshAll(position, velocity);
  // }

  /**
   * @return rotations per minute if a servo, meters per second if a linear or flywheel
   */
  @Override
  public double getEncoderVelocity() {
    // if (configuration.mode != MotorMode.kServo) {
    // // converts to meters
    // // BaseStatusSignal.refreshAll(talonFX.getPosition());
    // return velocity.getValue().in(RotationsPerSecond)
    // * Math.PI
    // * configuration.finalDiameterMeters;
    // }
    // // converts to RPM
    // return velocity.getValue().in(RotationsPerSecond) * 60.0;
    return 0;
  }

  /**
   * @return rotations per minute if a servo, meters per second if a linear or flywheel
   */
  public double getEncoderVelocity(StatusSignal<AngularVelocity> velocity) {
    if (configuration.mode != MotorMode.kServo) {
      // converts to meters
      // BaseStatusSignal.refreshAll(talonFX.getPosition());
      return velocity.getValue().in(RotationsPerSecond)
          * Math.PI
          * configuration.finalDiameterMeters;
    }
    // converts to RPM
    return velocity.getValue().in(RotationsPerSecond) * 60.0;
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
    return talonFX.getClosedLoopError().getValue();

    // if (configuration.mode == MotorMode.kFlywheel) {
    // return setpoint - getEncoderVelocity();
    // }
    // return setpoint - getEncoderPosition();
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
        // maybe its * instead of /
      case kFlywheel:
        this.setpoint = setpoint / (Math.PI * configuration.finalDiameterMeters);
        talonFX.setControl(new VelocityVoltage(this.setpoint).withEnableFOC(withFOC));
        break;
      case kServo:
        this.setpoint = setpoint;
        talonFX.setControl(new PositionVoltage(setpoint).withEnableFOC(withFOC));
        DriverStation.reportWarning(
            "Warning: TalonFX motor with the id "
                + talonFX.getDeviceID()
                + " is a Servo set with a double setpoint./n Use Rotation2d instead.",
            false);
      case kLinear:
        this.setpoint = setpoint / (Math.PI * configuration.finalDiameterMeters);
        talonFX.setControl(new PositionDutyCycle(this.setpoint).withEnableFOC(withFOC));
        break;
      default:
        this.setpoint = setpoint;
        talonFX.setControl(new VelocityVoltage(setpoint).withEnableFOC(withFOC));
        break;
    }
  }

  // /**
  // *
  // @param setpoint - Radians setpoint
  // */
  // @Override
  // public void setSetpoint(double setpoint) {
  // setSetpoint(new Rotation2d(setpoint));
  // }
  @Override
  public MotorConfiguration getMotorConfiguration() {
    return this.configuration;
  }

  public boolean isUsingTorqueCurrentFOC() {
    return this.useTorqueCurrentFOC;
  }

  public void useTorqueCurrentFOC(boolean using) {
    this.useTorqueCurrentFOC = using;
  }
}
