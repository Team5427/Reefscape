package team5427.frc.robot.subsystems.Swerve.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.SwerveConstants;

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveMotor;
  private final DCMotorSim steerMotor;

  private static final DCMotor driveMotorGearbox = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor steerMotorGearbox = DCMotor.getKrakenX60Foc(1);

  // private final PIDController driveController;
  // private final PIDController steerController;

  private SwerveModuleState targetModuleState;

  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double steerAppliedVolts = 0.0;
  public final int index;

  public ModuleIOSim(int index) {
    this.index = index;
    driveMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveMotorGearbox,
                0.01,
                SwerveConstants.kDriveMotorConfiguration.gearRatio.getMathematicalGearRatio()),
            driveMotorGearbox);
    steerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                steerMotorGearbox,
                0.003,
                SwerveConstants.kSteerMotorConfiguration.gearRatio.getMathematicalGearRatio()),
            steerMotorGearbox);
    // steerController =
    // new PIDController(
    // SimulationConstants.steerkP, SimulationConstants.steerkI,
    // SimulationConstants.steerkD);
    // driveController =
    // new PIDController(
    // SimulationConstants.drivekP, SimulationConstants.drivekI,
    // SimulationConstants.drivekD);
    // SwerveConstants.kSIMSteerController.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    driveMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.6, 12.6));
    steerMotor.setInputVoltage(MathUtil.clamp(steerAppliedVolts, -12.6, 12.6));
    driveMotor.update(Constants.kLoopSpeed);
    steerMotor.update(Constants.kLoopSpeed);

    inputs.currentModuleState =
        new SwerveModuleState(
            driveMotor.getAngularVelocityRPM()
                * Math.PI
                * SwerveConstants.kWheelDiameterMeters
                / 60.0,
            Rotation2d.fromRotations(
                steerMotor.getAngularPositionRotations()
                    - Math.floor(steerMotor.getAngularPositionRotations())));
    inputs.driveMotorPosition = Rotation2d.fromRotations(driveMotor.getAngularPositionRotations());
    inputs.steerMotorVelocityRotations =
        RotationsPerSecond.of(steerMotor.getAngularVelocityRPM() / 60.0);

    inputs.steerPosition =
        Rotation2d.fromRotations(
            steerMotor.getAngularPositionRotations()
                - Math.floor(steerMotor.getAngularPositionRotations()));

    inputs.currentModulePosition =
        new SwerveModulePosition(
            driveMotor.getAngularPositionRotations()
                * Math.PI
                * SwerveConstants.kWheelDiameterMeters,
            inputs.steerPosition);
    inputs.driveMotorVoltage = Volts.of(driveMotor.getInputVoltage());
    inputs.steerMotorVoltage = Volts.of(steerMotor.getInputVoltage());

    inputs.driveMotorConnected = true;
    inputs.steerMotorConnected = true;

    inputs.driveMotorCurrent = Amps.of(driveMotor.getCurrentDrawAmps());
    inputs.steerMotorCurrent = Amps.of(steerMotor.getCurrentDrawAmps());

    inputs.absolutePosition = inputs.steerPosition;
    inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryDrivePositionsMeters =
        new double[] {
          inputs.driveMotorPosition.getRotations() * SwerveConstants.kWheelDiameterMeters * Math.PI
        };
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.steerPosition};
  }

  /**
   * @param speed meters per second
   */
  @Override
  public void setDriveSpeedSetpoint(LinearVelocity speed) {
    driveFFVolts = SwerveConstants.kSIMDriveFeedforward.calculate(speed.in(MetersPerSecond));
    SwerveConstants.kSIMDriveController.setSetpoint(speed.in(MetersPerSecond));
    driveAppliedVolts =
        driveFFVolts
            + SwerveConstants.kSIMDriveController.calculate(
                driveMotor.getAngularVelocityRadPerSec() * SwerveConstants.kWheelDiameterMeters);
  }

  /*
   * Needs a voltage
   */
  @Override
  public void setDriveSpeedSetpoint(Voltage volts) {
    driveAppliedVolts = volts.in(Volts);
  }

  @Override
  public void setSteerPositionSetpoint(Rotation2d position) {
    SwerveConstants.kSIMSteerController.setSetpoint(position.getRotations());
    steerAppliedVolts =
        SwerveConstants.kSIMSteerController.calculate(
            steerMotor.getAngularPositionRotations()
                - Math.floor(steerMotor.getAngularPositionRotations()));
  }

  @Override
  public void setDriveSpeedSetpoint(Current current) {
    driveAppliedVolts = current.in(Amps);
  }

  @Override
  public void setSteerPositionSetpoint(Current current) {
    steerAppliedVolts = current.in(Amps);
  }

  /*
   * Needs a voltage
   */
  @Override
  public void setSteerPositionSetpoint(Voltage volts) {
    steerAppliedVolts = volts.in(Volts);
  }

  @Override
  public void setModuleState(SwerveModuleState state) {
    targetModuleState = state;
    setDriveSpeedSetpoint(MetersPerSecond.of(state.speedMetersPerSecond));
    setSteerPositionSetpoint(state.angle);
  }

  public void resetMotorSetpoint() {
    driveMotor.setState(0, 0);
    steerMotor.setAngle(0);
  }

  @Override
  public void stop() {
    driveMotor.setInputVoltage(0.0);
    steerMotor.setInputVoltage(0.0);
  }
}
