package team5427.frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.Constants.VisionConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Vision.io.VisionIO;
import team5427.frc.robot.subsystems.Vision.io.VisionIO.PoseObservation;
import team5427.frc.robot.subsystems.Vision.io.VisionIO.PoseObservationType;
import team5427.frc.robot.subsystems.Vision.io.VisionIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Vision.io.VisionIOPhoton;
import team5427.frc.robot.subsystems.Vision.io.VisionIOPhotonSim;
import team5427.lib.detection.tuples.Tuple2Plus;

public class VisionSubsystem extends SubsystemBase {
  private VisionIO[] io = new VisionIO[VisionConstants.kCameraCount];
  private VisionIOInputsAutoLogged[] inputsAutoLogged =
      new VisionIOInputsAutoLogged[VisionConstants.kCameraCount];
  private Pose3d referencePose;
  private final Alert[] disconnectedAlerts;

  private final VisionConsumer visionConsumer;

  private static VisionSubsystem m_instance;

  @Getter private Pose3d latestPoseMeasurement;

  public static VisionSubsystem getInstance(
      Optional<VisionConsumer> consumer,
      Optional<Supplier<Pose2d>> referencePoseSupplier,
      Optional<Supplier<Tuple2Plus<Double, Rotation2d>>> referenceHeadingSupplier) {
    if (m_instance == null) {
      if (consumer.isEmpty()
          || referencePoseSupplier.isEmpty()
          || referenceHeadingSupplier.isEmpty()) {
        DriverStation.reportWarning("Vision Subsystem Not provided Vision Consumer", true);
        return null;
      }
      m_instance =
          new VisionSubsystem(
              consumer.get(), referencePoseSupplier.get(), referenceHeadingSupplier.get());
    }
    return m_instance;
  }

  public static VisionSubsystem getInstance() {
    if (m_instance == null) {

      DriverStation.reportWarning("Vision Subsystem Not provided Vision Consumer", true);
      return null;
    }
    return m_instance;
  }

  private VisionSubsystem(
      VisionConsumer consumer,
      Supplier<Pose2d> referencePoseSupplier,
      Supplier<Tuple2Plus<Double, Rotation2d>> referenceHeadingSupplier) {
    super();
    switch (Constants.currentMode) {
      case REAL:
        io[0] =
            new VisionIOPhoton(
                VisionConstants.kSwerveCamName,
                VisionConstants.kSwerveCamTransform,
                referencePoseSupplier,
                referenceHeadingSupplier);
        io[1] =
            new VisionIOPhoton(
                VisionConstants.kIntakeCamName,
                VisionConstants.kIntakeCamTransform,
                referencePoseSupplier,
                referenceHeadingSupplier);
        // io[2] = new VisionIOQuestNav(VisionConstants.kQuestCameraTransform);

        for (int i = 0; i < inputsAutoLogged.length; i++) {
          inputsAutoLogged[i] = new VisionIOInputsAutoLogged();
        }
        break;
      case REPLAY:
      case SIM:
        io[0] =
            new VisionIOPhotonSim(
                VisionConstants.kSwerveCamName,
                VisionConstants.kSwerveCamTransform,
                referencePoseSupplier,
                referenceHeadingSupplier);
        io[1] =
            new VisionIOPhoton(
                VisionConstants.kIntakeCamName,
                VisionConstants.kIntakeCamTransform,
                referencePoseSupplier,
                referenceHeadingSupplier);
        for (int i = 0; i < inputsAutoLogged.length; i++) {
          inputsAutoLogged[i] = new VisionIOInputsAutoLogged();
        }
      default:
        break;
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputsAutoLogged.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
    this.visionConsumer = consumer;
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return Rotation2d.fromDegrees(inputsAutoLogged[cameraIndex].poseObservations[0].tx());
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputsAutoLogged[i]);
      Logger.processInputs("Vision/Camera " + Integer.toString(i), inputsAutoLogged[i]);
    }

    // Initialize logging values
    // List<Pose3d> allTagPoses = new LinkedList<>();
    // List<Pose3d> allRobotPoses = new LinkedList<>();
    // List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    // List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    int n = 0;

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Initialize logging values
      // List<Pose3d> tagPoses = new LinkedList<>();
      // List<Pose3d> robotPoses = new LinkedList<>();
      // List<Pose3d> robotPosesAccepted = new LinkedList<>();
      // List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      // for (int tagId : inputsAutoLogged[cameraIndex].tagIds) {
      //   var tagPose = VisionConstants.kAprilTagLayout.getTagPose(tagId);
      //   if (tagPose.isPresent()) {
      //     tagPoses.add(tagPose.get());
      //   }
      // }

      // Loop over pose observations
      for (PoseObservation observation : inputsAutoLogged[cameraIndex].poseObservations) {
        n++;
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > VisionConstants.kMaxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > VisionConstants.kMaxZHeight.in(Meter) // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > VisionConstants.kAprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > VisionConstants.kAprilTagLayout.getFieldWidth()
                // Must not be an impossible pose to acheive based on max drivetrain speeds
                || observation
                        .pose()
                        .toPose2d()
                        .relativeTo(RobotState.getInstance().getAdaptivePose())
                        .getTranslation()
                        .getNorm()
                    > SwerveConstants.kDriveMotorConfiguration.maxVelocity
                        * 3.5
                        * (Timer.getTimestamp() - observation.timestamp());

        // Add pose to log
        Logger.recordOutput("Vision Pose " + cameraIndex, observation.pose());

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            observation.type().equals(PoseObservationType.PHOTONVISION_SINGLE_TAG)
                ? observation.averageTagDistance() * 2.0
                : Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = VisionConstants.kLinearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstants.kAngularStdDevBaseline * stdDevFactor;
        if (cameraIndex < VisionConstants.kCameraStdDevFactors.length) {
          linearStdDev *= VisionConstants.kCameraStdDevFactors[cameraIndex];
          angularStdDev *= VisionConstants.kCameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        visionConsumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        latestPoseMeasurement = observation.pose();
      }

      // Log camera datadata
      // Logger.recordOutput(
      //     "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
      //     tagPoses.toArray(new Pose3d[tagPoses.size()]));
      // Logger.recordOutput(
      //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
      //     robotPoses.toArray(new Pose3d[robotPoses.size()]));
      // Logger.recordOutput(
      //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
      //     robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      // Logger.recordOutput(
      //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
      //     robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      // allTagPoses.addAll(tagPoses);
      // allRobotPoses.addAll(robotPoses);
      // allRobotPosesAccepted.addAll(robotPosesAccepted);
      // allRobotPosesRejected.addAll(robotPosesRejected);
    }

    Logger.recordOutput("Num of Pose Observations", n);
    n = 0;
    // Log summary data
    if (!DriverStation.isFMSAttached()) {
      // Logger.recordOutput(
      //     "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
      // Logger.recordOutput(
      //     "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
      // Logger.recordOutput(
      //     "Vision/Summary/RobotPosesAccepted",
      //     allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
      // Logger.recordOutput(
      //     "Vision/Summary/RobotPosesRejected",
      //     allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
  }

  // public boolean isQuestConnected() {
  //   return inputsAutoLogged[inputsAutoLogged.length - 1].connected;
  // }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
