package team5427.frc.robot.subsystems.Swerve.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public Rotation2d absolutePosition = new Rotation2d(0);
        public Rotation2d steerPosition = new Rotation2d(0);
        public SwerveModuleState currentModuleState = new SwerveModuleState(0, absolutePosition);
        public SwerveModuleState targetModuleState = new SwerveModuleState(0, absolutePosition);

        public SwerveModulePosition currentModulePosition = new SwerveModulePosition(0, absolutePosition);
        public Rotation2d driveMotorPosition = new Rotation2d(0);
        public double steerMotorVelocityRotationsPerSecond = 0.0;

        public double driveMotorVoltage = 0.0;
        public double steerMotorVoltage = 0.0;

        public double driveMotorCurrent = 0.0;
        public double steerMotorCurrent = 0.0;

        public boolean driveMotorConnected = false;
        public boolean steerMotorConnected = false;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};


    }

    public default void updateInputs(ModuleIOInputs inputs) {
    }

    public default void setDriveSpeedSetpoint(Double speed) {
    }

    /*
     * Needs a voltage
     */
    public default void setDriveSpeedSetpoint(double volts) {
    }

    public default void setSteerPositionSetpoint(Rotation2d position) {
    }

    /*
     * Needs a voltage
     */
    public default void setSteerPositionSetpoint(double position) {
    }

    public default void setModuleState(SwerveModuleState state) {
    }

    public default void resetMotorSetpoint(){}

}