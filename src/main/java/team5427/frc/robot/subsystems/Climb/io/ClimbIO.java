package team5427.frc.robot.subsystems.Climb.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    public Rotation2d hookPosition = Rotation2d.kZero;
    public AngularVelocity hookVelocity = RotationsPerSecond.of(0.0);
    public AngularAcceleration hookAcceleration = RotationsPerSecondPerSecond.of(0.0);

    public Current hookServoCurrent = Amps.of(0.0);
    public Voltage hookServoVoltage = Volts.of(0.0);
  }

  public void setHookSetpoint(Rotation2d setpoint);
}
