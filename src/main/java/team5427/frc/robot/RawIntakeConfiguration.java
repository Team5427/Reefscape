package team5427.frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class RawIntakeConfiguration {

  private final Rotation2d cascadeAngle;
  private final Distance cascadeHeight;
  private final Rotation2d wristAngle;
  private final LinearVelocity rollerSpeeds;
  private final boolean isCoral;

  public RawIntakeConfiguration() {
    cascadeAngle = wristAngle = Rotation2d.kZero;
    cascadeHeight = Meters.of(0.0);
    rollerSpeeds = MetersPerSecond.of(0.0);
    isCoral = true;
  }

  public RawIntakeConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      LinearVelocity rollerSpeeds,
      boolean isCoral) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.rollerSpeeds = rollerSpeeds;
    this.isCoral = isCoral;
  }

  public Rotation2d getCascadeAngle() {
    return cascadeAngle;
  }

  public Distance getCascadeHeight() {
    return cascadeHeight;
  }

  public Rotation2d getWristAngle() {
    return wristAngle;
  }

  public LinearVelocity getRollerSpeeds() {
    return rollerSpeeds;
  }

  public boolean isCoral() {
    return isCoral;
  }
}
