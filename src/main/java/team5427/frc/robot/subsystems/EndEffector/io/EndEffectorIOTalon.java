package team5427.frc.robot.subsystems.EndEffector.io;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import team5427.frc.robot.Constants.EndEffectorConstants;
import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.SteelTalonFX;

public class EndEffectorIOTalon implements EndEffectorIO {

  private MagicSteelTalonFX pivotMotor;
  private MagicSteelTalonFX wristMotor;
  private SteelTalonFX coralRollerMotor;
  private SteelTalonFX algaeRollerMotor;

  private CANcoder wristCancoder;
  private CANcoder pivotCancoder;

  public EndEffectorIOTalon() {
    pivotMotor = new MagicSteelTalonFX(EndEffectorConstants.kPivotMotorCanID);
    wristMotor = new MagicSteelTalonFX(EndEffectorConstants.kWristMotorCanID);
    coralRollerMotor = new SteelTalonFX(EndEffectorConstants.kCoralRollerMotorCanID);
    algaeRollerMotor = new SteelTalonFX(EndEffectorConstants.kAlgaeRollerMotorCanID);

    pivotMotor.apply(EndEffectorConstants.kPivotMotorConfiguration);
    wristMotor.apply(EndEffectorConstants.kWristMotorConfiguration);
    coralRollerMotor.apply(EndEffectorConstants.kCoralRollerMotorConfiguration);
    algaeRollerMotor.apply(EndEffectorConstants.kAlgaeRollerMotorConfiguration);

    wristCancoder = new CANcoder(EndEffectorConstants.kCoralRollerMotorCanID.getDeviceNumber());
    CANcoderConfiguration wristConfig = new CANcoderConfiguration();

    wristConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);
    wristConfig.MagnetSensor.MagnetOffset = EndEffectorConstants.kWristCancoderOffset;
    wristConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristCancoder.getConfigurator().apply(wristConfig);

    wristCancoder.clearStickyFaults();

    pivotCancoder = new CANcoder(EndEffectorConstants.kCoralRollerMotorCanID.getDeviceNumber());
    CANcoderConfiguration pivotConfig = new CANcoderConfiguration();

    pivotConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);
    pivotConfig.MagnetSensor.MagnetOffset = EndEffectorConstants.kPivotCancoderOffset;
    pivotConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    pivotCancoder.getConfigurator().apply(pivotConfig);

    pivotCancoder.clearStickyFaults();

    wristMotor.talonConfig.Feedback.FeedbackRemoteSensorID = wristCancoder.getDeviceID();
    wristMotor.talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristMotor.talonConfig.Feedback.SensorToMechanismRatio = 1.0;
    wristMotor.talonConfig.Feedback.RotorToSensorRatio =
        EndEffectorConstants.kWristMotorConfiguration.gearRatio.getMathematicalGearRatio();
    wristMotor.getTalonFX().getConfigurator().apply(wristMotor.talonConfig);

    pivotMotor.talonConfig.Feedback.FeedbackRemoteSensorID = pivotCancoder.getDeviceID();
    pivotMotor.talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotMotor.talonConfig.Feedback.SensorToMechanismRatio = 1.0;
    pivotMotor.talonConfig.Feedback.RotorToSensorRatio =
        EndEffectorConstants.kPivotMotorConfiguration.gearRatio.getMathematicalGearRatio();
    pivotMotor.getTalonFX().getConfigurator().apply(pivotMotor.talonConfig);
  }

  @Override
  public void setCoralRollerSetpoint(LinearVelocity velocity) {
    coralRollerMotor.setSetpoint(velocity.in(MetersPerSecond));
  }

  @Override
  public void setCoralWristSetpoint(Rotation2d setpoint) {
    wristMotor.setSetpoint(setpoint);
  }

  @Override
  public void setAlgaeRollerSetpoint(LinearVelocity velocity) {
    algaeRollerMotor.setSetpoint(velocity.in(MetersPerSecond));
  }

  @Override
  public void setPivotSetpoint(Rotation2d setpoint) {
    pivotMotor.setSetpoint(setpoint);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.algaeRollerMotorConnected = algaeRollerMotor.getTalonFX().isConnected();

    inputs.algaeRollerMotorVoltage = algaeRollerMotor.getTalonFX().getMotorVoltage().getValue();
    inputs.algaeRollerMotorCurrent = algaeRollerMotor.getTalonFX().getSupplyCurrent().getValue();

    inputs.algaeRollerMotorLinearVelocity =
        MetersPerSecond.of(algaeRollerMotor.getEncoderVelocity());
    inputs.algaeRollerMotorAngularVelocity = algaeRollerMotor.getTalonFX().getVelocity().getValue();
    inputs.algaeRollerMotorAngularAcceleration =
        algaeRollerMotor.getTalonFX().getAcceleration().getValue();

    inputs.coralRollerMotorConnected = coralRollerMotor.getTalonFX().isConnected();

    inputs.coralRollerMotorVoltage = coralRollerMotor.getTalonFX().getMotorVoltage().getValue();
    inputs.coralRollerMotorCurrent = coralRollerMotor.getTalonFX().getSupplyCurrent().getValue();

    inputs.coralRollerMotorLinearVelocity =
        MetersPerSecond.of(coralRollerMotor.getEncoderVelocity());
    inputs.coralRollerMotorAngularVelocity = coralRollerMotor.getTalonFX().getVelocity().getValue();
    inputs.coralRollerMotorAngularAcceleration =
        coralRollerMotor.getTalonFX().getAcceleration().getValue();

    inputs.pivotMotorConnected = pivotMotor.getTalonFX().isConnected();

    inputs.pivotMotorCurrent = pivotMotor.getTalonFX().getStatorCurrent().getValue();
    inputs.pivotMotorVoltage = pivotMotor.getTalonFX().getMotorVoltage().getValue();

    inputs.pivotMotorAngularAcceleration = pivotMotor.getTalonFX().getAcceleration().getValue();
    inputs.pivotMotorAngularVelocity =
        RotationsPerSecond.of(pivotMotor.getEncoderVelocity() / 60.0);
    inputs.pivotAngle = Rotation2d.fromRotations(pivotMotor.getEncoderPosition());

    inputs.wristMotorConnected = wristMotor.getTalonFX().isConnected();

    inputs.wristMotorVoltage = wristMotor.getTalonFX().getMotorVoltage().getValue();
    inputs.wristMotorCurrent = wristMotor.getTalonFX().getStatorCurrent().getValue();

    inputs.wristMotorAngularAcceleration = wristMotor.getTalonFX().getAcceleration().getValue();
    inputs.wristMotorAngularVelocity =
        RotationsPerSecond.of(wristMotor.getEncoderVelocity() / 60.0);
    inputs.wristAngle = Rotation2d.fromRotations(wristMotor.getEncoderPosition());
  }
}
