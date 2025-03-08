package team5427.frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.Swerve.io.ModuleIO;
import team5427.frc.robot.subsystems.Swerve.io.ModuleIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Swerve.io.ModuleIOSim;
import team5427.frc.robot.subsystems.Swerve.io.ModuleIOTalon;

public class SwerveModule {
  private ModuleIO io;
  private int index;
  public ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  // Don't know if we need to use turn relative offset if we have cancoder module
  // offsets
  public Rotation2d turnRelativeOffset = null;

  public SwerveModule(int index) {
    this.index = index;
    // TODO
    switch (Constants.currentMode) {
      case SIM:
        io = new ModuleIOSim(index);
        break;
      case REAL:
        io = new ModuleIOTalon(index);
        break;
      case REPLAY:
        break;
      default:
        break;
    }
    inputs.currentModuleState = new SwerveModuleState(0, getCancoderRotation());

    // io.resetMotorSetpoint();
  }

  public ModuleIO getModuleIO() {
    return io;
  }

  public void setModuleState(SwerveModuleState state) {
    SwerveModuleState newState = state;
    switch (Constants.currentMode) {
      case REPLAY:
      case REAL:
        if (io != null) {
          newState.optimize(inputs.absolutePosition);
          io.setModuleState(newState);
        }
        break;
      case SIM:
        if (io != null) {
          io.setModuleState(newState);
        }
        break;
      default:
        break;
    }
  }

  public  void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsMeters[i];
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  public SwerveModulePosition getModulePosition() {
    return inputs.currentModulePosition;
  }

  public SwerveModuleState getModuleState() {
    return inputs.currentModuleState;
  }

  public Rotation2d getCancoderRotation() {
    return inputs.absolutePosition;
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getSimAngle() {
    if (turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.steerPosition.plus(turnRelativeOffset);
    }
  }

  /*
   * Returns the drive motor position in rotations
   */
  public double drivePosition() {
    return inputs.driveMotorPosition.getRotations();
  }

  public void stop() {
    io.stop();
  }
}
