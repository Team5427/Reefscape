package team5427.frc.robot.subsystems.Vision.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];

    // public double[] timestamps = new double[] {};
    // public double[][] frames = new double[][] {};

    public int[] tagIds = new int[0];
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double tx,
      double ty,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION_MULTI_TAG,
    PHOTONVISION_SINGLE_TAG,
    QUEST_NAV,
  }

  public void updateInputs(VisionIOInputs inputs);

  public default void setReferencePose(Pose3d resetPose) {}

  public default void setLastPose(Pose3d lastPose) {}

  public default void setPipeline(int pipelineNumber) {}

  public void applyCameraTransformation(Transform3d transformation);

  public default void setStdDev(Matrix<N3, N1> stddev) {}
}
