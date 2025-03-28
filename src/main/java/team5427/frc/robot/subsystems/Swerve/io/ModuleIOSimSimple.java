package team5427.frc.robot.subsystems.Swerve.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

public class ModuleIOSimSimple implements ModuleIO {
  private final DCMotorSim driveMotor;
  private final DCMotorSim steerMotor;

  private static final DCMotor driveMotorGearbox = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor steerMotorGearbox = DCMotor.getKrakenX60Foc(1);

  // private final PIDController driveController;
  // private final PIDController steerController;

  private SwerveModuleState targetModuleState;
  public final int index;

  public ModuleIOSimSimple(int index) {
    this.index = index;
    driveMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveMotorGearbox,
                0.0001,
                SwerveConstants.kDriveMotorConfiguration.gearRatio.getMathematicalGearRatio()),
            driveMotorGearbox);
    steerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                steerMotorGearbox,
                0.0003,
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

    driveMotor.setAngularVelocity(
        targetModuleState.speedMetersPerSecond / (SwerveConstants.kWheelDiameterMeters));
    steerMotor.setAngle(targetModuleState.angle.getRadians());
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
  public void setDriveSpeedSetpoint(LinearVelocity speed) {}

  /*
   * Needs a voltage
   */
  @Override
  public void setDriveSpeedSetpoint(Voltage volts) {}

  @Override
  public void setSteerPositionSetpoint(Rotation2d position) {}

  @Override
  public void setDriveSpeedSetpoint(Current current) {}

  @Override
  public void setSteerPositionSetpoint(Current current) {}

  /*
   * Needs a voltage
   */
  @Override
  public void setSteerPositionSetpoint(Voltage volts) {}

  @Override
  public void setModuleState(SwerveModuleState state) {
    targetModuleState = state;
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
