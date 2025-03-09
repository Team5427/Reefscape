package team5427.frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.Map;
import team5427.frc.robot.FieldConstants.ReefHeight;

public class RawScoringConfiguration {

  private final Rotation2d cascadeAngle;
  private final Distance cascadeHeight;
  private final Rotation2d wristAngle;

  private Map<ReefHeight, Pose3d> scoringPose;

  private ReefHeight reefHeight;

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
      Map<ReefHeight, Pose3d> scoringPose) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = scoringPose;
  }

  public RawScoringConfiguration(
      Rotation2d cascadeAngle,
      Distance cascadeHeight,
      Rotation2d wristAngle,
      Map<ReefHeight, Pose3d> scoringPose,
      ReefHeight height) {
    this.cascadeAngle = cascadeAngle;
    this.cascadeHeight = cascadeHeight;
    this.wristAngle = wristAngle;
    this.scoringPose = scoringPose;
    this.reefHeight = height;
  }

  public RawScoringConfiguration setScoringPoses(Map<ReefHeight, Pose3d> map) {
    this.scoringPose = map;
    return this;
  }

  public RawScoringConfiguration setReefHeight(ReefHeight height) {
    this.reefHeight = height;
    return this;
  }

  public Pose3d getPose(ReefHeight height) {
    return scoringPose.get(height);
  }

  public Pose3d getPose() {
    return scoringPose.get(reefHeight);
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

  public Map<ReefHeight, Pose3d> getScoringPose() {
    return scoringPose;
  }
}
