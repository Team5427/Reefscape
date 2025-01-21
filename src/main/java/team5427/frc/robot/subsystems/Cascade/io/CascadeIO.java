package team5427.frc.robot.subsystems.Cascade.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface CascadeIO {
	@AutoLog
	public static class CascadeIOInputs {

		public LinearVelocity velocity = MetersPerSecond.of(0.0);
		public AngularVelocity velocityRotations = RotationsPerSecond.of(
				0.0);
		public Distance cascadeHeightMeters = Meters.of(0.0);
		public LinearAcceleration accelerationd = MetersPerSecondPerSecond.of(0.0);

		public Current cascadeMasterMotorCurrent = Amps.of(0.0);
		public Voltage cascadeMasterMotorVoltage = Volts.of(0.0);

		public Current cascadeSlaveMotorCurrent = Amps.of(0.0);
		public Voltage cascadeSlaveMotorVoltage = Volts.of(0.0);
	}

	public void updateInputs(CascadeIOInputs inputs);

	public void setCascadeSetpoint(double meters);

	public default void setCascadekG(double kG) {
	}
}
