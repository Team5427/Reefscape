package team5427.frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.Getter;

public class RawIntakeConfiguration {

  @Getter private final Rotation2d cascadeAngle;
  @Getter private final Distance cascadeHeight;
  @Getter private final Rotation2d wristAngle;
  @Getter private final LinearVelocity rollerSpeeds;
  @Getter private final boolean isCoral;

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
}
