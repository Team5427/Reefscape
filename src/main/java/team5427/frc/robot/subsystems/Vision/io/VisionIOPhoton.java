package team5427.frc.robot.subsystems.Vision.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import team5427.frc.robot.Constants.VisionConstants;

public class VisionIOPhoton implements VisionIO {

  private PhotonCamera cam;

  public Matrix<N3, N1> stddev;

  PhotonPoseEstimator photonPoseEstimator;

  public VisionIOPhoton(String cameraName, Transform3d cameraTransform) {
    cam = new PhotonCamera(cameraName);

    photonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraTransform);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = cam.isConnected();
    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    List<PoseObservation> obs = new ArrayList<PoseObservation>();

      for (int i = results.size() - 1; i > 0; i--) {
      if (results.get(i).multitagResult.isPresent()) {
        Pose3d pose =
            new Pose3d(
                results.get(i).getMultiTagResult().get().estimatedPose.best.getTranslation(),
                results.get(i).getMultiTagResult().get().estimatedPose.best.getRotation());
        double totalTagDistance = 0.0;
        Set<Short> tagIdSet = new HashSet<>();
        for (PhotonTrackedTarget target : results.get(i).targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          tagIdSet.add((short) target.fiducialId);
        }
        inputs.tagIds = new int[tagIdSet.size()];
        for (int j = 0; j < tagIdSet.size(); j++) {
          inputs.tagIds[j] = (Short) tagIdSet.toArray()[j];
        }

        obs.add(
            new PoseObservation(
                results.get(i).getTimestampSeconds(),
                pose,
                results.get(i).multitagResult.get().estimatedPose.ambiguity,
                results.get(i).multitagResult.get().fiducialIDsUsed.size(),
                totalTagDistance / results.get(i).targets.size(),
                results.get(i).getBestTarget().getYaw(),
                results.get(i).getBestTarget().getPitch(),
                PoseObservationType.PHOTONVISION));
        inputs.timestamps = Arrays.copyOf(inputs.timestamps, inputs.timestamps.length + 1);
        inputs.timestamps[inputs.timestamps.length-1] = results.get(i).getTimestampSeconds();
      } else {
        List<PhotonTrackedTarget> targets = results.get(i).getTargets();
        for (PhotonTrackedTarget target : targets) {
          Pose3d pose =
              new Pose3d(
                  target.bestCameraToTarget.getTranslation(),
                  target.bestCameraToTarget.getRotation());
          obs.add(
              new PoseObservation(
                  results.get(i).getTimestampSeconds(),
                  pose,
                  target.getPoseAmbiguity(),
                  1,
                  target.bestCameraToTarget.getTranslation().getNorm(),
                  results.get(i).getBestTarget().getYaw(),
                  results.get(i).getBestTarget().getPitch(),
                  PoseObservationType.PHOTONVISION));
          inputs.timestamps = Arrays.copyOf(inputs.timestamps, inputs.timestamps.length + 1);
          inputs.timestamps[inputs.timestamps.length-1] = results.get(i).getTimestampSeconds();
        }
      }
      List<PoseObservation> temp = Arrays.asList(inputs.poseObservations);
      temp = new ArrayList<PoseObservation>(temp);
      temp.addAll(obs);

      inputs.poseObservations = new PoseObservation[temp.size()];

      for(int b = 0; b <= temp.size()-1; b++){
        inputs.poseObservations[b] = temp.get(b);
      }
      
    }
  }

  @Override
  public void applyCameraTransformation(Transform3d transformation) {
    photonPoseEstimator.setRobotToCameraTransform(transformation);
  }

  @Override
  public void setStdDev(Matrix<N3, N1> stddev) {
    this.stddev = stddev;
  }
}
