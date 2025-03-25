package team5427.frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.Map;
import lombok.Getter;
import team5427.frc.robot.Field.ReefLevel;

public class RawScoringConfiguration {

  @Getter private final Rotation2d cascadeAngle;
  @Getter private final Distance cascadeHeight;
  @Getter private final Rotation2d wristAngle;
  @Getter private final LinearVelocity kCoralRollerVelocity;
  @Getter private final LinearVelocity kAlgaeRollerVelocity;

  private Map<ReefLevel, Pose3d> scoringPose;

  private ReefLevel reefHeight;

  public RawScoringConfiguration() {
    this.cascadeAngle = Rotation2d.kZero;
    this.cascadeHeight = Meters.of(0.0);
    this.wristAngle = Rotation2d.kZero;
    this.scoringPose = Map.of();
    this.kCoralRollerVelocity = MetersPerSecond.of(0.0);
    this.kAlgaeRollerVelocity = MetersPerSecond.of(0.0);
  }

  public RawScoringConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      LinearVelocity coralRollerVelocity,
      LinearVelocity algaeRollerVelocity) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = Map.of();
    this.kCoralRollerVelocity = coralRollerVelocity;
    this.kAlgaeRollerVelocity = algaeRollerVelocity;
  }

  public RawScoringConfiguration(
      Rotation2d cascadeAngle, Distance cascadeHeight, Rotation2d wristAngle) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = Map.of();
    this.kCoralRollerVelocity = MetersPerSecond.of(0.0);
    this.kAlgaeRollerVelocity = MetersPerSecond.of(0.0);
  }

  public RawScoringConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      Map<ReefLevel, Pose3d> scoringPose,
      LinearVelocity coralRollerVelocity,
      LinearVelocity algaeRollerVelocity) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = scoringPose;
    this.kCoralRollerVelocity = coralRollerVelocity;
    this.kAlgaeRollerVelocity = algaeRollerVelocity;
  }

  public RawScoringConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      Map<ReefLevel, Pose3d> scoringPose,
      ReefLevel height) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = scoringPose;
    this.reefHeight = height;
    this.kCoralRollerVelocity = MetersPerSecond.of(0.0);
    this.kAlgaeRollerVelocity = MetersPerSecond.of(0.0);
  }

  public RawScoringConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      Map<ReefLevel, Pose3d> scoringPose,
      ReefLevel height,
      LinearVelocity coralRollerVelocity,
      LinearVelocity algaeRollerVelocity) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = scoringPose;
    this.reefHeight = height;
    this.kCoralRollerVelocity = coralRollerVelocity;
    this.kAlgaeRollerVelocity = algaeRollerVelocity;
  }

  public RawScoringConfiguration setScoringPoses(Map<ReefLevel, Pose3d> map) {
    this.scoringPose = map;
    return this;
  }

  public RawScoringConfiguration setReefHeight(ReefLevel height) {
    this.reefHeight = height;
    return this;
  }

  public Pose3d getPose(ReefLevel height) {
    return scoringPose.get(height);
  }

  public Pose3d getPose() {
    return scoringPose.get(reefHeight);
  }

  public Map<ReefLevel, Pose3d> getScoringPose() {
    return scoringPose;
  }
}
