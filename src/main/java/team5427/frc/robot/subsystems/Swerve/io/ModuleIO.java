package team5427.frc.robot.subsystems.Swerve.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public Rotation2d absolutePosition = new Rotation2d(0);
    public Rotation2d steerPosition = new Rotation2d(0);
    public SwerveModuleState currentModuleState = new SwerveModuleState(0, absolutePosition);
    public SwerveModulePosition currentModulePosition =
        new SwerveModulePosition(0, absolutePosition);
    public Rotation2d driveMotorPosition = new Rotation2d(0);
    public AngularVelocity steerMotorVelocityRotations = RotationsPerSecond.of(0.0);

    public Voltage driveMotorVoltage = Volts.of(0.0);
    public Voltage steerMotorVoltage = Volts.of(0.0);

    public Current driveMotorCurrent = Amps.of(0.0);
    public Current steerMotorCurrent = Amps.of(0.0);

    public boolean driveMotorConnected = false;
    public boolean steerMotorConnected = false;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveSpeedSetpoint(LinearVelocity speed) {}

  /*
   * Needs a voltage
   */
  public default void setDriveSpeedSetpoint(Voltage volts) {}

  public default void setSteerPositionSetpoint(Rotation2d position) {}

  /*
   * Needs a voltage
   */
  public default void setSteerPositionSetpoint(Voltage volts) {}

  public default void setModuleState(SwerveModuleState state) {}

  public default void resetMotorSetpoint(Rotation2d steerPosition) {}
}
