package team5427.frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.lib.detection.tuples.Tuple2Plus;

public class RobotState {
  public SwerveDriveOdometry odometry;
  public SwerveDrivePoseEstimator poseEstimator;

  private Pose2d estimatedPose = new Pose2d();
  private Pose2d odometryPose = new Pose2d();

  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.3, 0.3, 0.001));

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public RobotState() {
    this.odometry =
        new SwerveDriveOdometry(
            SwerveConstants.m_kinematics, Rotation2d.kZero, new SwerveModulePosition[]{new SwerveModulePosition(0, Rotation2d.kZero),new SwerveModulePosition(0, Rotation2d.kZero),new SwerveModulePosition(0, Rotation2d.kZero),new SwerveModulePosition(0, Rotation2d.kZero)});
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.m_kinematics,
            Rotation2d.kZero,
            new SwerveModulePosition[]{new SwerveModulePosition(0, Rotation2d.kZero),new SwerveModulePosition(0, Rotation2d.kZero),new SwerveModulePosition(0, Rotation2d.kZero),new SwerveModulePosition(0, Rotation2d.kZero)},
            Pose2d.kZero,
            odometryStateStdDevs,
            VecBuilder.fill(0.4, 0.4, 9));
  }

  public void addOdometryMeasurement(
      double timestamp, Rotation2d rotation, SwerveModulePosition[] modulePoseModulePositions) {
    odometry.update(rotation, modulePoseModulePositions);
    poseEstimator.updateWithTime(timestamp, rotation, modulePoseModulePositions);
  }

  /**
   * Adds a new timestamped vision measurement. Meant to be a bridge method, actual vision
   * processing should be done in a seperate subsystem
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public Pose2d getOdometryPose() {
    this.odometryPose = odometry.getPoseMeters();
    return this.odometryPose;
  }

  public Pose2d getEstimatedPose() {
    this.estimatedPose = poseEstimator.getEstimatedPosition();
    return this.estimatedPose;
  }

  public void resetAllPose(Pose2d resetPose) {
    resetOdometryPose(resetPose);
    resetEstimatedPose(resetPose);
  }

  public void resetAllPose(
      Pose2d resetPose, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
    resetOdometryPose(resetPose, modulePositions, gyroAngle);
    resetEstimatedPose(resetPose, modulePositions, gyroAngle);
  }

  public void resetOdometryPose(Pose2d resetPose) {
    this.odometry.resetPose(resetPose);
  }

  public void resetOdometryPose(
      Pose2d resetPose, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
    this.odometry.resetPosition(gyroAngle, modulePositions, resetPose);
  }

  public void resetEstimatedPose(Pose2d resetPose) {
    this.poseEstimator.resetPose(resetPose);
  }

  public void resetEstimatedPose(
      Pose2d resetPose, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
    this.poseEstimator.resetPosition(gyroAngle, modulePositions, resetPose);
  }

  public void resetHeading(Rotation2d heading) {
    this.odometry.resetRotation(heading);
    this.poseEstimator.resetRotation(heading);
  }

  public Tuple2Plus<Double, Rotation2d> getOdometryHeading() {
    return new Tuple2Plus<Double, Rotation2d>(
        Timer.getTimestamp(), this.odometry.getPoseMeters().getRotation());
  }

  public void log() {
    Logger.recordOutput(
        "Localization/Estimation/Robot", RobotState.getInstance().getEstimatedPose());
    Logger.recordOutput("Localization/Odometry/Robot", RobotState.getInstance().getOdometryPose());
  }
}
