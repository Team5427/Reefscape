package team5427.frc.robot.subsystems.ProngEffector.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ProngIO {

  @AutoLog
  public static class ProngIOInputs {

    public Rotation2d wristPosition = Rotation2d.kZero;
    public AngularVelocity wristVelocity = RotationsPerSecond.of(0.0);
    public AngularAcceleration wristAcceleration = RotationsPerSecondPerSecond.of(0.0);

    public Current wristCurrent = Amps.of(0.0);
    public Voltage wristVoltage = Volts.of(0.0);

    public Rotation2d rollerPosition = Rotation2d.kZero;
    public LinearVelocity rollerVelocity = MetersPerSecond.of(0.0);
    public LinearAcceleration rollerAcceleration = MetersPerSecondPerSecond.of(0.0);

    public Current rollerCurrent = Amps.of(0.0);
    public Voltage rollerVoltage = Volts.of(0.0);
  }

  public void updateInputs(ProngIOInputs inputs);

  public void setWristSetpoint(Rotation2d setpoint);

  public void setRollerSpeeds(LinearVelocity velocity);

  public void stopRollers();
}
