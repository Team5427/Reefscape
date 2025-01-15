package team5427.frc.robot.subsystems.Cascade.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;

public interface CascadeIO {
    @AutoLog
    public static class CascadeIOInputs {
        public double velocityMetersPerSecond = 0.0;
        public double velocityRotationsPerSecond = 0.0;
        public double cascadeHeightMeters = 0.0;
        public double accelerationMetersPerSecondSquared = 0.0;

        public double cascadeMasterMotorCurrent;
        public double cascadeMasterMotorVoltage;

        public double cascadeSlaveMotorCurrent;
        public double cascadeSlaveMotorVoltage;
    }

    public void updateInputs(CascadeIOInputs inputs);

    public void setCascadeSetpoint(double meters);

    public default void setCascadekG(double kG) {
    }

}