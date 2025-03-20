package team5427.frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.Map;
import lombok.Getter;
import team5427.frc.robot.Field.ReefLevel;

public class RawScoringConfiguration {

  @Getter private final Rotation2d cascadeAngle;
  @Getter private final Distance cascadeHeight;
  @Getter private final Rotation2d wristAngle;

  private Map<ReefLevel, Pose3d> scoringPose;

  private ReefLevel reefHeight;

  public RawScoringConfiguration() {
    this.cascadeAngle = Rotation2d.kZero;
    this.cascadeHeight = Meters.of(0.0);
    this.wristAngle = Rotation2d.kZero;
    this.scoringPose = Map.of();
  }

  public RawScoringConfiguration(
      Rotation2d cascadeAngle, Distance cascadeHeight, Rotation2d wristAngle) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = Map.of();
  }

  public RawScoringConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      Map<ReefLevel, Pose3d> scoringPose) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = scoringPose;
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
