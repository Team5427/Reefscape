package team5427.lib.motors.real;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.IMotorController;
import team5427.lib.motors.real.MotorConfiguration.IdleState;
import team5427.lib.motors.real.MotorConfiguration.MotorMode;

public class SimpleSparkFlex implements IMotorController {

  private CANDeviceId id;
  private SparkFlex sparkFlex;
  private MotorConfiguration configuration;
  private RelativeEncoder relativeEncoder;
  private SparkClosedLoopController controller;
  private SparkFlexConfig config;
  private SparkBase.ControlType controlType;

  private double setpoint;

  public SimpleSparkFlex(CANDeviceId id) {
    this.id = id;
    sparkFlex = new SparkFlex(id.getDeviceNumber(), MotorType.kBrushless);

    relativeEncoder = sparkFlex.getEncoder();
    // relativeEncoder.setMeasurementPeriod(10);
    config = new SparkFlexConfig();
    controller = sparkFlex.getClosedLoopController();
  }

  @Override
  public void apply(MotorConfiguration configuration) {
    this.configuration = configuration;
    config
        .inverted(configuration.isInverted)
        .idleMode(configuration.idleState == IdleState.kBrake ? IdleMode.kBrake : IdleMode.kCoast);

    // sparkMax.setInverted(configuration.isInverted);
    // sparkMax.setIdleMode(configuration.idleState == IdleState.kBrake ?
    // IdleMode.kBrake : IdleMode.kCoast);

    // config.encoder.positionConversionFactor(configuration.unitConversionRatio)
    //         .velocityConversionFactor(configuration.unitConversionRatio / 60.0);

    // relativeEncoder.setPositionConversionFactor(configuration.unitConversionRatio);
    // relativeEncoder.setVelocityConversionFactor(configuration.unitConversionRatio
    // / 60.0);

    config.closedLoop.pidf(configuration.kP, configuration.kI, configuration.kD, configuration.kFF);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // controller.setP(configuration.kP);
    // controller.setI(configuration.kI);
    // controller.setD(configuration.kD);
    // controller.setFF(configuration.kFF);

    switch (configuration.mode) {
      case kFlywheel:
        controlType = SparkBase.ControlType.kVelocity;
        break;
      case kServo:
      case kLinear:
        controlType = SparkBase.ControlType.kPosition;
        // configured in rotations rather than radians
        config
            .closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(-0.5)
            .positionWrappingMaxInput(0.5);
        // controller.setPositionPIDWrappingEnabled(true);
        // controller.setPositionPIDWrappingMinInput(-Math.PI);
        // controller.setPositionPIDWrappingMaxInput(Math.PI);
        break;
      default:
        controlType = SparkBase.ControlType.kVelocity;
        break;
    }

    sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // sparkMax.burnFlash();
    // sparkFlex.setInverted(configuration.isInverted);
    // sparkFlex.setIdleMode(configuration.idleState == IdleState.kBrake ? IdleMode.kBrake :
    // IdleMode.kCoast);

    // relativeEncoder.setPositionConversionFactor(configuration.unitConversionRatio);
    // relativeEncoder.setVelocityConversionFactor(configuration.unitConversionRatio / 60.0);

    // controller.setP(configuration.kP);
    // controller.setI(configuration.kI);
    // controller.setD(configuration.kD);
    // controller.setFF(configuration.kFF);

    // switch (configuration.mode) {
    //     case kFlywheel:
    //         controlType = CANSparkBase.ControlType.kVelocity;
    //         break;
    //     case kServo:
    //     case kLinear:
    //         controlType = CANSparkBase.ControlType.kPosition;
    //         controller.setPositionPIDWrappingEnabled(true);
    //         controller.setPositionPIDWrappingMinInput(-Math.PI);
    //         controller.setPositionPIDWrappingMaxInput(Math.PI);
    //         break;
    //     default:
    //         controlType = CANSparkBase.ControlType.kVelocity;
    //         break;
    // }

    // sparkFlex.burnFlash();
  }

  /**
   * @param setpoint needs to be in meters if a flywheel or linear, or rotations if a servo
   * @apiNote If the device is a servo, it is recommended to use Rotation2d, rather than a double
   */
  @Override
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
    controller.setReference(this.setpoint, controlType);
  }

  /*
   * Function now uses rotations inside rather than radians
   */
  @Override
  public void setSetpoint(Rotation2d setpoint) {
    this.setpoint = setpoint.getRotations();
    if (configuration.mode == MotorMode.kFlywheel) {
      DriverStation.reportWarning(
          "Simple Spark Max of id "
              + id.getDeviceNumber()
              + " of type flywheel was set with Rotation2d setpoint.",
          true);
    }

    controller.setReference(this.setpoint, controlType);
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * @param position - In rotations
   */
  @Override
  public void setEncoderPosition(double position) {
    relativeEncoder.setPosition(position);
  }

  @Override
  public void setEncoderPosition(Rotation2d position) {
    /**
     * Position set in radians as PositionConversionFactor is already applied Which converts
     * rotations to radians by default
     */
    relativeEncoder.setPosition(position.getRotations());
  }

  @Override
  public double getEncoderPosition() {
    return relativeEncoder.getPosition();
  }

  /**
   * @return RPM of the motor
   */
  @Override
  public double getEncoderVelocity() {
    return relativeEncoder.getVelocity();
  }

  @Override
  public void setRawPercentage(double percentage) {
    sparkFlex.set(percentage);
  }

  @Override
  public void setRelativePercentage(double percentage) {
    sparkFlex.setVoltage(percentage * sparkFlex.getBusVoltage());
  }

  @Override
  public void setRawVoltage(double voltage) {
    sparkFlex.setVoltage(voltage);
  }

  @Override
  public double getError() {
    if (configuration.mode == MotorMode.kFlywheel) {
      return setpoint - getEncoderVelocity();
    }
    return setpoint - getEncoderPosition();
  }

  public SparkFlex getSparkFlex() {
    return sparkFlex;
  }

  public RelativeEncoder getRelativeEncoder() {
    return relativeEncoder;
  }

  @Override
  public MotorConfiguration getMotorConfiguration() {
    return this.configuration;
  }
}
