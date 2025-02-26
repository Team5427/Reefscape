package team5427.frc.robot.subsystems.Cascade.io;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.MotorConfiguration;

public class CascadeIOMagicTalon implements CascadeIO {

  private MagicSteelTalonFX cascadeMotorMaster;
  private MagicSteelTalonFX cascadeMotorSlave;
  // private MotorConfiguration cascadeConfig;

  private MagicSteelTalonFX pivotMotorMaster;
  private MagicSteelTalonFX pivotMotorSlave;
  private MotorConfiguration pivotConfig;
  private CANcoder pivotCANcoder;

  private StatusSignal<AngularVelocity> cascadeRotVelocity;
  private StatusSignal<AngularAcceleration> cascadeRotAccel;
  private StatusSignal<Current> cascadeMasterMotorCurrent;
  private StatusSignal<Voltage> cascadeMasterMotorVoltage;
  private StatusSignal<Current> cascadeSlaveMotorCurrent;
  private StatusSignal<Voltage> cascadeSlaveMotorVoltage;

  private StatusSignal<Angle> pivotPosition;
  private StatusSignal<Angle> absolutePivotPosition;
  private StatusSignal<AngularVelocity> pivotRotVelocity;
  private StatusSignal<AngularAcceleration> pivotRotAccel;
  private StatusSignal<Current> pivotMasterMotorCurrent;
  private StatusSignal<Voltage> pivotMasterMotorVoltage;
  private StatusSignal<Current> pivotSlaveMotorCurrent;
  private StatusSignal<Voltage> pivotSlaveMotorVoltage;

  public boolean cascadeMotorsStopped;
  public boolean pivotMotorsStopped;

  public CascadeIOMagicTalon() {
    cascadeMotorMaster = new MagicSteelTalonFX(CascadeConstants.kCascadeMasterId);
    cascadeMotorSlave = new MagicSteelTalonFX(CascadeConstants.kCascadeSlaveId);
    cascadeMotorMaster.apply(CascadeConstants.kCascadeDriverConfiguration);
    MotorConfiguration cascadeSlaveConfig =
        new MotorConfiguration(CascadeConstants.kCascadeDriverConfiguration);
    // cascadeSlaveConfig.isInverted = !CascadeConstants.kCascadeDriverConfiguration.isInverted;
    cascadeMotorSlave.apply(cascadeSlaveConfig);

    cascadeMotorSlave
        .getTalonFX()
        .setControl(new Follower(cascadeMotorMaster.getTalonFX().getDeviceID(), true));

    pivotMotorMaster = new MagicSteelTalonFX(CascadeConstants.kPivotMasterId);
    pivotMotorSlave = new MagicSteelTalonFX(CascadeConstants.kPivotSlaveId);
    pivotMotorMaster.apply(CascadeConstants.kPivotConfiguration);

    MotorConfiguration pivotSlaveConfig =
        new MotorConfiguration(CascadeConstants.kPivotConfiguration);
    pivotMotorSlave.apply(pivotSlaveConfig);

    pivotMotorSlave
        .getTalonFX()
        .setControl(new Follower(pivotMotorMaster.getTalonFX().getDeviceID(), true));

    pivotCANcoder =
        new CANcoder(
            CascadeConstants.kPivotCANcoderId.getDeviceNumber(),
            CascadeConstants.kPivotCANcoderId.getBus());

    CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();
    pivotEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    pivotEncoderConfig.MagnetSensor.MagnetOffset = CascadeConstants.kPivotCancoderOffset;
    pivotEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    pivotCANcoder.getConfigurator().apply(pivotEncoderConfig);

    cascadeMotorMaster.useTorqueCurrentFOC(true);
    cascadeMotorSlave.useTorqueCurrentFOC(true);
    pivotMotorMaster.useTorqueCurrentFOC(true);
    pivotMotorSlave.useTorqueCurrentFOC(true);

    cascadeRotVelocity = cascadeMotorMaster.getTalonFX().getVelocity();
    cascadeRotAccel = cascadeMotorMaster.getTalonFX().getAcceleration();

    cascadeMasterMotorCurrent = cascadeMotorMaster.getTalonFX().getStatorCurrent();
    cascadeMasterMotorVoltage = cascadeMotorMaster.getTalonFX().getMotorVoltage();
    cascadeSlaveMotorCurrent = cascadeMotorSlave.getTalonFX().getStatorCurrent();
    cascadeSlaveMotorVoltage = cascadeMotorSlave.getTalonFX().getMotorVoltage();

    pivotPosition = pivotMotorMaster.getTalonFX().getPosition();
    absolutePivotPosition = pivotCANcoder.getAbsolutePosition();
    pivotRotVelocity = pivotMotorMaster.getTalonFX().getVelocity();
    pivotRotAccel = pivotMotorMaster.getTalonFX().getAcceleration();

    pivotMasterMotorCurrent = pivotMotorMaster.getTalonFX().getStatorCurrent();
    pivotMasterMotorVoltage = pivotMotorMaster.getTalonFX().getMotorVoltage();
    pivotSlaveMotorCurrent = pivotMotorSlave.getTalonFX().getStatorCurrent();
    pivotSlaveMotorVoltage = pivotMotorSlave.getTalonFX().getMotorVoltage();

    pivotMotorMaster.talonConfig.Feedback.FeedbackRemoteSensorID = pivotCANcoder.getDeviceID();
    pivotMotorMaster.talonConfig.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue.FusedCANcoder;
    pivotMotorMaster.talonConfig.Feedback.SensorToMechanismRatio = 1.0;
    pivotMotorMaster.talonConfig.Feedback.RotorToSensorRatio =
        CascadeConstants.kPivotConfiguration.gearRatio.getSensorToMechanismRatio();
    pivotMotorMaster.getTalonFX().getConfigurator().apply(pivotMotorMaster.talonConfig);

    cascadeMotorMaster.setEncoderPosition(0.0);
    cascadeMotorSlave.setEncoderPosition(0.0);
    pivotMotorMaster.setEncoderPosition(
        Rotation2d.fromRotations(absolutePivotPosition.refresh().getValue().in(Rotations)));
    pivotMotorMaster.setEncoderPosition(
        Rotation2d.fromRotations(absolutePivotPosition.refresh().getValue().in(Rotations)));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotRotVelocity,
        pivotRotAccel,
        pivotMasterMotorCurrent,
        pivotMasterMotorVoltage,
        pivotSlaveMotorCurrent,
        pivotSlaveMotorVoltage);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        cascadeRotVelocity,
        cascadeRotAccel,
        cascadeMasterMotorCurrent,
        cascadeMasterMotorVoltage,
        cascadeSlaveMotorCurrent,
        cascadeSlaveMotorVoltage);

    BaseStatusSignal.waitForAll(
        0.02,
        pivotPosition,
        pivotRotVelocity,
        pivotRotAccel,
        pivotMasterMotorCurrent,
        pivotMasterMotorVoltage,
        pivotSlaveMotorCurrent,
        pivotSlaveMotorVoltage,
        absolutePivotPosition);

    BaseStatusSignal.waitForAll(
        0.02,
        cascadeRotVelocity,
        cascadeRotAccel,
        cascadeMasterMotorCurrent,
        cascadeMasterMotorVoltage,
        cascadeSlaveMotorCurrent,
        cascadeSlaveMotorVoltage);

    ParentDevice.optimizeBusUtilizationForAll(
        cascadeMotorMaster.getTalonFX(), cascadeMotorSlave.getTalonFX());
    ParentDevice.optimizeBusUtilizationForAll(
        pivotMotorMaster.getTalonFX(), pivotMotorSlave.getTalonFX(), pivotCANcoder);
  }

  @Override
  public void updateInputs(CascadeIOInputs inputs) {

    // Logger.recordOutput("Pivot Brake Mode", cascadeMotorMaster.getTalonFX());
    // Logger.recordOutput("Cascade Motor Reader Setpoint", cascadeMotorMaster.getSetpoint());

    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotRotVelocity,
        pivotRotAccel,
        pivotMasterMotorCurrent,
        pivotMasterMotorVoltage,
        pivotSlaveMotorCurrent,
        pivotSlaveMotorVoltage,
        absolutePivotPosition);

    BaseStatusSignal.refreshAll(
        cascadeRotVelocity,
        cascadeRotAccel,
        cascadeMasterMotorCurrent,
        cascadeMasterMotorVoltage,
        cascadeSlaveMotorCurrent,
        cascadeSlaveMotorVoltage);

    // Logger.recordOutput("Cascade PID Output", cascadeMotorMaster.getError());
    Logger.recordOutput("Pivot Setpoint", pivotMotorMaster.getSetpoint());

    inputs.velocity = MetersPerSecond.of(cascadeMotorMaster.getEncoderVelocity(cascadeRotVelocity));
    inputs.velocityRotations = cascadeRotVelocity.getValue();
    inputs.cascadeHeightMeters =
        Meters.of(
            cascadeMotorMaster.getEncoderPosition(cascadeMotorMaster.getTalonFX().getPosition()));
    inputs.acceleration =
        MetersPerSecondPerSecond.of(
            cascadeRotAccel.getValue().in(RotationsPerSecondPerSecond)
                * Math.PI
                * CascadeConstants.kCascadeDriverConfiguration.finalDiameterMeters);

    inputs.cascadeMasterMotorCurrent = cascadeMasterMotorCurrent.getValue();
    inputs.cascadeMasterMotorVoltage = cascadeMasterMotorVoltage.getValue();

    inputs.cascadeSlaveMotorCurrent = cascadeSlaveMotorCurrent.getValue();
    inputs.cascadeSlaveMotorVoltage = cascadeSlaveMotorVoltage.getValue();

    inputs.pivotRotation = Rotation2d.fromRotations(pivotPosition.getValue().in(Rotations));
    inputs.pivotRotationVelocity = pivotRotVelocity.getValue();
    inputs.pivotRotationAcceleration = pivotRotAccel.getValue();

    inputs.absolutePivotRotation =
        Rotation2d.fromRotations(absolutePivotPosition.getValue().in(Rotations));
  }

  @Override
  public void setCascadeSetpoint(Distance setpoint) {
    // cascade motor must calculate the kG outside of the motor controller as it is
    // on a pivoting arm
    if (cascadeMotorsStopped) {
      cascadeMotorMaster.getTalonFX().set(0);
      return;
    }
    // cascadeMotorMaster.getMotorConfiguration().kFF =
    // CascadeConstants.kCascadeDriverConfiguration.isArm
    //         ? CascadeConstants.kCascadeDriverGravityFF
    //                 * pivotCANcoder.getAbsolutePosition().getValue().in(Rotations)
    //         : CascadeConstants.kCascadeDriverGravityFF;
    cascadeMotorMaster.setSetpoint(setpoint.in(Meters));
  }

  @Override
  public void setCascadeEncoderPosition(Distance setpoint) {
    cascadeMotorMaster.setEncoderPosition(setpoint.magnitude());
  }

  @Override
  public void setPivotSetpoint(Rotation2d setpoint) {
    if (pivotMotorsStopped) {
      pivotMotorMaster.getTalonFX().set(0);
      return;
    }
    pivotMotorMaster.setSetpoint(setpoint);
  }

  @Override
  public void setCANCoderPosition(Rotation2d angle) {
    pivotCANcoder.setPosition(angle.getMeasure());
  }

  @Override
  public void stopCascadeMotors(boolean stopped) {
    this.cascadeMotorsStopped = stopped;
  }

  @Override
  public void stopPivotMotors(boolean stopped) {
    this.pivotMotorsStopped = stopped;
  }
}
