package team5427.frc.robot.subsystems.SupremeEffector.io;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface SupremeIO {
  @AutoLog
  public class SupremeIOInputs {

    public boolean pivotMotorConnected = false;
    public boolean coralMotorConnected = false;
    public boolean algaeMotorConnected = false;

    public Current pivotMotorStatorCurrent = Amps.of(0.0);
    public Current coralMotorStatorCurrent = Amps.of(0.0);
    public Current algaeMotorStatorCurrent = Amp.of(0.0);

    public Voltage pivotMotorVoltage = Volts.of(0.0);
    public Voltage coralMotorVoltage = Volts.of(0.0);
    public Voltage algaeMotorVoltage = Volts.of(0.0);

    public Rotation2d pivotMotorPosition = new Rotation2d();
    public Distance coralMotorPosition = Meters.of(0.0);
    public Distance algaeMotorPosition = Meters.of(0.0);

    public AngularVelocity pivotMotorAngularVelocity = RotationsPerSecond.of(0.0);
    public LinearVelocity coralMotorLinearVelocity = MetersPerSecond.of(0.0);
    public LinearVelocity algaeMotorLinearVelocity = MetersPerSecond.of(0.0);

    public AngularAcceleration pivotMotorAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);
    public LinearAcceleration coralMotorLinearAcceleration = MetersPerSecondPerSecond.of(0.0);
    public LinearAcceleration algaeMotorLinearAcceleration = MetersPerSecondPerSecond.of(0.0);
  }

  public void updateInputs(SupremeIOInputs inputs);

  public void setPivotMotorSetpoint(Rotation2d angle);

  public void setCoralMotorSetpoint(LinearVelocity velocity);

  public void setAlgaeMotorSetpoint(LinearVelocity velocity);

  public void setPivotMotorPosition(Rotation2d position);
}
