package team5427.frc.robot.subsystems.Swerve.io;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.PhoenixOdometryThread;
import team5427.lib.motors.real.SteelTalonFX;

public class ModuleIOTalon implements ModuleIO {

  private SteelTalonFX steerMotor;
  private SteelTalonFX driveMotor;

  private CANcoder cancoder;

  private final int moduleIdx;

  private SwerveModuleState targetModuleState;
  private StatusSignal<Voltage> steerMotorVoltage;
  private StatusSignal<Voltage> driveMotorVoltage;
  private StatusSignal<Angle> absolutePosition;
  private StatusSignal<Current> steerMotorCurrent;
  private StatusSignal<Current> driveMotorCurrent;
  private StatusSignal<Angle> driveMotorPosition;
  private StatusSignal<Angle> steerMotorPosition;
  private StatusSignal<AngularVelocity> driveMotorVelocity;
  private StatusSignal<AngularVelocity> steerMotorVelocity;

  private Queue<Double> drivePositionQueue;
  private Queue<Double> steerPositionQueue;

  private ProfiledPIDController rotationController;

  private final Queue<Double> timestampQueue;

  public ModuleIOTalon(int moduleIdx) {

    this.moduleIdx = moduleIdx;

    driveMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[moduleIdx]);
    steerMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[moduleIdx]);
    cancoder =
        new CANcoder(
            SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getDeviceNumber(),
            SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getBus());
    SwerveConstants.kDriveMotorConfiguration.isInverted =
        SwerveConstants.kSwerveUtilInstance.kDriveInversion[moduleIdx];
    SwerveConstants.kSteerMotorConfiguration.isInverted =
        SwerveConstants.kSwerveUtilInstance.kSteerInversion[moduleIdx];
    driveMotor.apply(SwerveConstants.kDriveMotorConfiguration);
    steerMotor.apply(SwerveConstants.kSteerMotorConfiguration);

    driveMotor.getTalonFX().clearStickyFaults();
    steerMotor.getTalonFX().clearStickyFaults();

    CANcoderConfiguration configuration = new CANcoderConfiguration();
    configuration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
    configuration.MagnetSensor.MagnetOffset =
        SwerveConstants.kSwerveUtilInstance.kModuleOffsets[moduleIdx];
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(configuration);

    cancoder.clearStickyFaults();
    absolutePosition = cancoder.getAbsolutePosition();

    steerMotor.talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    steerMotor.talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerMotor.talonConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerMotor.talonConfig.Feedback.RotorToSensorRatio =
        SwerveConstants.kSteerMotorConfiguration.gearRatio.getSensorToMechanismRatio();
    steerMotor.getTalonFX().getConfigurator().apply(steerMotor.talonConfig);

    steerMotor.setEncoderPosition(absolutePosition.refresh().getValue().in(Rotations));
    driveMotor.setEncoderPosition(0.0);
    driveMotor.useTorqueCurrentFOC(true);

    driveMotorPosition = driveMotor.getTalonFX().getPosition();
    steerMotorPosition = steerMotor.getTalonFX().getPosition();

    driveMotorVelocity = driveMotor.getTalonFX().getVelocity();
    steerMotorVelocity = steerMotor.getTalonFX().getVelocity();

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getTalonFX().getPosition());

    steerPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(steerMotor.getTalonFX().getPosition());

    System.out.println("New Module with idx: " + moduleIdx);

    driveMotorVoltage = driveMotor.getTalonFX().getMotorVoltage();
    steerMotorVoltage = steerMotor.getTalonFX().getMotorVoltage();
    driveMotorCurrent = driveMotor.getTalonFX().getStatorCurrent();
    steerMotorCurrent = steerMotor.getTalonFX().getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kOdometryFrequency,
        driveMotorPosition,
        steerMotorPosition
        
        );
        // steerMotorVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveMotorVoltage, steerMotorVoltage, driveMotorCurrent, steerMotorCurrent, absolutePosition,driveMotorVelocity);

    ParentDevice.optimizeBusUtilizationForAll(
        driveMotor.getTalonFX(), steerMotor.getTalonFX(), cancoder);

    BaseStatusSignal.waitForAll(
        0.02,
        absolutePosition,
        driveMotorPosition,
        steerMotorPosition,
        driveMotorVelocity);
        // steerMotorVoltage,
        // driveMotorVoltage,
        // driveMotorCurrent,
        // steerMotorCurrent);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // FIX ME

    // BaseStatusSignal.refreshAll(s
    //     absolutePosition,
    //     driveMotorPosition,
    //     steerMotorPosition,
    //     driveMotorVelocity,
    //     steerMotorVoltage,
    //     driveMotorVoltage,
    //     driveMotorCurrent,
    //     steerMotorCurrent);
    BaseStatusSignal.refreshAll(
        
        driveMotorPosition,
        steerMotorPosition
        );

    BaseStatusSignal.refreshAll(
        steerMotorVoltage, driveMotorVoltage, driveMotorCurrent, steerMotorCurrent, driveMotorVelocity, absolutePosition);

    inputs.absolutePosition = Rotation2d.fromRotations(absolutePosition.getValue().in(Rotations));
    // steerMotor.updateStatusSignals();
    // driveMotor.updateStatusSignals();

    inputs.driveMotorPosition =
        Rotation2d.fromRotations(driveMotor.getEncoderPosition(driveMotorPosition));
    inputs.steerMotorVelocityRotations =
        RotationsPerSecond.of(steerMotor.getEncoderVelocity(steerMotorVelocity) / 60.0);

    inputs.steerPosition =
        Rotation2d.fromRotations(steerMotor.getEncoderPosition(steerMotorPosition));

    inputs.currentModuleState =
        new SwerveModuleState(
            driveMotor.getEncoderVelocity(driveMotorVelocity), inputs.absolutePosition);
    inputs.currentModulePosition =
        new SwerveModulePosition(
            driveMotor.getEncoderPosition(driveMotorPosition), inputs.absolutePosition);

    inputs.driveMotorVoltage = driveMotorVoltage.getValue();
    inputs.steerMotorVoltage = steerMotorVoltage.getValue();

    inputs.driveMotorConnected = driveMotor.getTalonFX().isConnected();
    inputs.steerMotorConnected = steerMotor.getTalonFX().isConnected();

    inputs.driveMotorCurrent = driveMotorCurrent.getValue();
    inputs.steerMotorCurrent = steerMotorCurrent.getValue();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            // converts the raw rotations -> radians -> meters
            .mapToDouble((Double value) -> value * Math.PI * SwerveConstants.kWheelDiameterMeters)
            .toArray();
    inputs.odometryTurnPositions =
        steerPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    steerPositionQueue.clear();
  }

  @Override
  public void resetMotorSetpoint(Rotation2d steerPosition) {
    steerMotor.setEncoderPosition(steerPosition);
    driveMotor.setEncoderPosition(0.0);
  }

  /**
   * Needs to be a <strong>Double</strong>, NOT a double
   *
   * @param speed - <strong>Double</strong>, setpoint for drive motor (module speed)
   */
  @Override
  public void setDriveSpeedSetpoint(LinearVelocity speed) {
    driveMotor.setSetpoint(speed.in(MetersPerSecond));
  }

  @Override
  public void setDriveSpeedSetpoint(Voltage volts) {
    driveMotor.setRawVoltage(volts.in(Volt));
  }

  /**
   * @param setpoint - <strong>Rotation2d</strong> setpoint for steer motor (module angle)
   */
  @Override
  public void setSteerPositionSetpoint(Rotation2d setpoint) {
    steerMotor.setSetpoint(setpoint);
  }

  @Override
  public void setModuleState(SwerveModuleState state) {
    Rotation2d currentAngle = Rotation2d.fromRotations(absolutePosition.getValue().in(Rotations));
    state.optimize(currentAngle);
    state.cosineScale(currentAngle);
    targetModuleState = state;
    setDriveSpeedSetpoint(MetersPerSecond.of(targetModuleState.speedMetersPerSecond));
    setSteerPositionSetpoint(targetModuleState.angle);
  }

  @Override
  public void stop() {

    setSteerPositionSetpoint(Rotation2d.fromRotations(steerMotorPosition.getValue().in(Rotations)));

    setDriveSpeedSetpoint(MetersPerSecond.of(0.0));
    // driveMotor.getTalonFX().stopMotor();
    steerMotor.getTalonFX().set(0.0);
  }
}
