package team5427.frc.robot.subsystems.Cascade.io;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.checkerframework.checker.units.qual.A;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.MotorConfiguration;
import team5427.lib.motors.real.SteelTalonFX;

public class CascadeIOMagicTalon implements CascadeIO {

  private SteelTalonFX cascadeMotorMaster;
  private SteelTalonFX cascadeMotorSlave;
  private MotorConfiguration cascadeConfig;

  private MagicSteelTalonFX pivotMotorMaster;
  private MagicSteelTalonFX pivotMotorSlave;
  private MotorConfiguration pivotConfig;
  private CANcoder pivotCANcoder;

  private StatusSignal<AngularVelocity> cascadeRotVelocity;
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
    cascadeMotorMaster = new SteelTalonFX(CascadeConstants.kCascadeMasterId);
    cascadeMotorSlave = new SteelTalonFX(CascadeConstants.kCascadeSlaveId);
    cascadeMotorMaster.apply(CascadeConstants.kCascadeDriverConfiguration);
    MotorConfiguration cascadeSlaveConfig =
        new MotorConfiguration(CascadeConstants.kCascadeDriverConfiguration);
    cascadeSlaveConfig.isInverted = true;
    cascadeMotorSlave.apply(cascadeSlaveConfig);
    pivotMotorMaster = new MagicSteelTalonFX(CascadeConstants.kPivotMasterId);
    pivotMotorSlave = new MagicSteelTalonFX(CascadeConstants.kPivotSlaveId);
    pivotMotorMaster.apply(CascadeConstants.kPivotConfiguration);

    MotorConfiguration pivotSlaveConfig =
        new MotorConfiguration(CascadeConstants.kPivotConfiguration);
    pivotSlaveConfig.isInverted = true;
    pivotMotorSlave.apply(pivotSlaveConfig);
    pivotCANcoder = new CANcoder(CascadeConstants.kPivotCANcoderId.getDeviceNumber());
    pivotCANcoder.getConfigurator().apply(CascadeConstants.kPivotEncoderConfig);

    cascadeRotVelocity = cascadeMotorMaster.getTalonFX().getVelocity();
    
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

    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0, 
      cascadeRotVelocity,
      cascadeMasterMotorCurrent,
      cascadeMasterMotorVoltage,
      cascadeSlaveMotorCurrent,
      cascadeSlaveMotorVoltage,

      pivotPosition,
      pivotRotVelocity,
      pivotRotAccel,
      pivotMasterMotorCurrent,
      pivotMasterMotorVoltage,
      pivotSlaveMotorCurrent,
      pivotSlaveMotorVoltage
    );

    BaseStatusSignal.waitForAll(
      0.02, 
      cascadeRotVelocity,
      cascadeMasterMotorCurrent,
      cascadeMasterMotorVoltage,
      cascadeSlaveMotorCurrent,
      cascadeSlaveMotorVoltage,

      pivotPosition,
      pivotRotVelocity,
      pivotRotAccel,
      pivotMasterMotorCurrent,
      pivotMasterMotorVoltage,
      pivotSlaveMotorCurrent,
      pivotSlaveMotorVoltage
    );

    ParentDevice.optimizeBusUtilizationForAll(
      cascadeMotorMaster.getTalonFX(),
      pivotMotorMaster.getTalonFX()
    );

  }

  @Override
  public void updateInputs(CascadeIOInputs inputs) {

    BaseStatusSignal.refreshAll(
      cascadeRotVelocity,
      cascadeMasterMotorCurrent,
      cascadeMasterMotorVoltage,
      cascadeSlaveMotorCurrent,
      cascadeSlaveMotorVoltage,

      pivotPosition,
      pivotRotVelocity,
      pivotRotAccel,
      pivotMasterMotorCurrent,
      pivotMasterMotorVoltage,
      pivotSlaveMotorCurrent,
      pivotSlaveMotorVoltage
    );

    inputs.velocity = MetersPerSecond.of(cascadeMotorMaster.getEncoderVelocity());
    inputs.velocityRotations = cascadeRotVelocity.getValue();
    inputs.cascadeHeightMeters = Meters.of(cascadeMotorMaster.getEncoderPosition());
    inputs.acceleration = MetersPerSecondPerSecond.of(
            cascadeMotorMaster
                    .getTalonFX()
                    .getAcceleration()
                    .getValue()
                    .in(RotationsPerSecondPerSecond)
                * Math.PI
                * cascadeConfig.finalDiameterMeters);

    inputs.cascadeMasterMotorCurrent = cascadeMasterMotorCurrent.getValue();
    inputs.cascadeMasterMotorVoltage = cascadeMasterMotorVoltage.getValue();

    inputs.cascadeSlaveMotorCurrent = cascadeSlaveMotorCurrent.getValue();
    inputs.cascadeSlaveMotorVoltage = cascadeSlaveMotorVoltage.getValue();

    inputs.pivotRotation = Rotation2d.fromRotations(pivotPosition.getValue().in(Rotations));
    inputs.pivotRotationVelocity = pivotRotVelocity.getValue();
    inputs.pivotRotationAcceleration = pivotRotAccel.getValue();

    inputs.absolutePivotRotation = Rotation2d.fromRotations(absolutePivotPosition.getValue().in(Rotations));
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
