package team5427.frc.robot.subsystems.Vision.io;

import org.checkerframework.checker.units.qual.C;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionIOPhoton implements VisionIO {

    public PhotonCamera cam;

    public Matrix<N3, N1> stddev;

    public VisionIOPhoton(String cameraName) {
        cam = new PhotonCamera(cameraName);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setResetPose(Pose3d resetPose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setResetPose'");
    }

    @Override
    public void applyCameraTransformation(Pose3d transformation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'applyCameraTransformation'");
    }

}
