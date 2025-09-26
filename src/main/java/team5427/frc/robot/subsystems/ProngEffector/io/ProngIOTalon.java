package team5427.frc.robot.subsystems.ProngEffector.io;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.SteelTalonFX;

public class ProngIOTalon implements ProngIO {

  private MagicSteelTalonFX wristServo;
  private SteelTalonFX rollerTalon;

  private StatusSignal<Angle> wristPosition;
  private StatusSignal<AngularVelocity> wristVelocity;
  private StatusSignal<AngularAcceleration> wristAcceleration;

  private StatusSignal<Current> wristCurrent;
  private StatusSignal<Voltage> wristVoltage;

  private StatusSignal<Angle> rollerPosition;
  private StatusSignal<AngularVelocity> rollerVelocity;
  private StatusSignal<AngularAcceleration> rollerAcceleration;

  private StatusSignal<Current> rollerCurrent;
  private StatusSignal<Voltage> rollerVoltage;

  private StatusSignal<Temperature> rollerTemperature;

  public ProngIOTalon() {
    wristServo = new MagicSteelTalonFX(ProngEffectorConstants.kWristServoId);
    wristServo.apply(ProngEffectorConstants.kWristConfiguration);

    wristServo.talonConfig.ClosedLoopGeneral.ContinuousWrap = false;
    wristServo.getTalonFX().getConfigurator().apply(wristServo.talonConfig);

    rollerTalon = new SteelTalonFX(ProngEffectorConstants.kRollerServoId);
    rollerTalon.apply(ProngEffectorConstants.kRollerConfiguration);

    wristPosition = wristServo.getTalonFX().getPosition();
    wristVelocity = wristServo.getTalonFX().getVelocity();
    wristAcceleration = wristServo.getTalonFX().getAcceleration();

    wristCurrent = wristServo.getTalonFX().getStatorCurrent();
    wristVoltage = wristServo.getTalonFX().getMotorVoltage();

    rollerPosition = rollerTalon.getTalonFX().getPosition();
    rollerVelocity = rollerTalon.getTalonFX().getVelocity();
    rollerAcceleration = rollerTalon.getTalonFX().getAcceleration();

    rollerCurrent = rollerTalon.getTalonFX().getStatorCurrent();
    rollerVoltage = rollerTalon.getTalonFX().getMotorVoltage();

    rollerTemperature = rollerTalon.getTalonFX().getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(10, rollerTemperature);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        wristPosition,
        wristVelocity,
        wristAcceleration,
        wristCurrent,
        wristVoltage,
        rollerPosition,
        rollerVelocity,
        rollerAcceleration,
        rollerCurrent,
        rollerVoltage);

    BaseStatusSignal.waitForAll(
        0.02,
        wristPosition,
        wristVelocity,
        wristAcceleration,
        wristCurrent,
        wristVoltage,
        rollerPosition,
        rollerVelocity,
        rollerAcceleration,
        rollerCurrent,
        rollerVoltage);

    ParentDevice.optimizeBusUtilizationForAll(wristServo.getTalonFX(), rollerTalon.getTalonFX());
    wristServo.setEncoderPosition(ProngEffectorConstants.kZeroPosition);
  }

  @Override
  public void updateInputs(ProngIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        wristPosition,
        wristVelocity,
        wristAcceleration,
        wristCurrent,
        wristVoltage,
        rollerPosition,
        rollerVelocity,
        rollerAcceleration,
        rollerCurrent,
        rollerVoltage);

    inputs.wristPosition = Rotation2d.fromRotations(wristPosition.getValue().in(Rotations));
    inputs.wristVelocity = wristVelocity.getValue();
    inputs.wristAcceleration = wristAcceleration.getValue();

    inputs.wristCurrent = wristCurrent.getValue();
    inputs.wristVoltage = wristVoltage.getValue();

    inputs.rollerPosition = Rotation2d.fromRotations(rollerPosition.getValue().in(Rotations));
    inputs.rollerVelocity = MetersPerSecond.of(rollerTalon.getEncoderVelocity(rollerVelocity));
    inputs.rollerAcceleration =
        MetersPerSecondPerSecond.of(rollerTalon.getEncoderAcceleration(rollerAcceleration));

    inputs.rollerCurrent = rollerCurrent.getValue();
    inputs.rollerVoltage = rollerVoltage.getValue();
    inputs.rollerTemperature = rollerTemperature.getValue();
  }

  @Override
  public void setWristSetpoint(Rotation2d setpoint) {
    wristServo.setSetpoint(setpoint);
  }

  @Override
  public void setRollerSpeeds(LinearVelocity velocity) {
    rollerTalon.setSetpoint(velocity.in(MetersPerSecond));
  }

  @Override
  public void stopRollers() {
    rollerTalon.setSetpoint(0.0);
  }

  @Override
  public boolean hasResistance() {
    return rollerCurrent.getValue().magnitude()
        >= ProngEffectorConstants.kRollerConfiguration.currentLimit - 1;
  }
}
