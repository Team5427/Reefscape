package team5427.frc.robot.subsystems.EndEffector.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public boolean coralRollerMotorConnected = false;
    public boolean algaeRollerMotorConnected = false;

    public boolean wristMotorConnected = false;
    public boolean pivotMotorConnected = false;

    public Current coralRollerMotorCurrent = Amps.of(0.0);
    public Current algaeRollerMotorCurrent = Amps.of(0.0);
    public Current wristMotorCurrent = Amps.of(0.0);
    public Current pivotMotorCurrent = Amps.of(0.0);

    public Voltage coralRollerMotorVoltage = Volts.of(0.0);
    public Voltage algaeRollerMotorVoltage = Volts.of(0.0);
    public Voltage wristMotorVoltage = Volts.of(0.0);
    public Voltage pivotMotorVoltage = Volts.of(0.0);

    public AngularVelocity coralRollerMotorAngularVelocity = RotationsPerSecond.of(0.0);
    public AngularVelocity algaeRollerMotorAngularVelocity = RotationsPerSecond.of(0.0);
    public AngularVelocity wristMotorAngularVelocity = RotationsPerSecond.of(0.0);
    public AngularVelocity pivotMotorAngularVelocity = RotationsPerSecond.of(0.0);

    public LinearVelocity coralRollerMotorLinearVelocity = MetersPerSecond.of(0.0);
    public LinearVelocity algaeRollerMotorLinearVelocity = MetersPerSecond.of(0.0);

    public AngularAcceleration wristMotorAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);
    public AngularAcceleration pivotMotorAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);
    public AngularAcceleration coralRollerMotorAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);
    public AngularAcceleration algaeRollerMotorAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);

    public Rotation2d pivotAngle = new Rotation2d(0.0);
    public Rotation2d wristAngle = new Rotation2d(0.0);
  }

  public void updateInputs(EndEffectorIOInputs inputs);

  public void setCoralRollerSetpoint(LinearVelocity velocity);

  public void setCoralWristSetpoint(Rotation2d setpoint);

  public void setAlgaeRollerSetpoint(LinearVelocity velocity);

  public void setPivotSetpoint(Rotation2d setpoint);

  // public void stopCoralWristMotor(boolean stopped);

  // public void stopPivotMotor(boolean stopped);

  // public void stopAlgaeRollerMotor(boolean stopped);

  // public void stopCoralRollerMotor(boolean stopped);
}
