package team5427.lib.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import team5427.lib.motors.real.MotorConfiguration;

public interface IMotorController {

  public void apply(MotorConfiguration configuration);

  public MotorConfiguration getMotorConfiguration();

  public void setSetpoint(double setpoint);

  public void setSetpoint(Rotation2d setpoint);

  public double getSetpoint();

  public void setEncoderPosition(double position);

  public void setEncoderPosition(Rotation2d position);

  public double getEncoderPosition();

  public double getEncoderVelocity();

  public double getError();

  public void setRawPercentage(double percentage);

  public void setRelativePercentage(double percentage);

  public void setRawVoltage(double voltage);
}
