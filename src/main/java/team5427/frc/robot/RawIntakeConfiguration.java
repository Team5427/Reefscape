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

  /** This is always for the coral motor */
  @Getter private final LinearVelocity coralRollerSpeeds;

  @Getter private final LinearVelocity algaeRollerSpeeds;
  @Getter private final boolean isCoral;

  public RawIntakeConfiguration() {
    cascadeAngle = wristAngle = Rotation2d.kZero;
    cascadeHeight = Meters.of(0.0);
    coralRollerSpeeds = MetersPerSecond.of(0.0);
    this.algaeRollerSpeeds = MetersPerSecond.of(0.0);
    isCoral = true;
  }

  public RawIntakeConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      LinearVelocity coralRollerSpeeds,
      LinearVelocity algaeRollerSpeeds,
      boolean isCoral) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.coralRollerSpeeds = coralRollerSpeeds;
    this.algaeRollerSpeeds = algaeRollerSpeeds;
    this.isCoral = isCoral;
  }

  public RawIntakeConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      LinearVelocity coralRollerSpeeds,
      boolean isCoral) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.coralRollerSpeeds = coralRollerSpeeds;
    this.algaeRollerSpeeds = MetersPerSecond.of(0.0);
    this.isCoral = isCoral;
  }
}
