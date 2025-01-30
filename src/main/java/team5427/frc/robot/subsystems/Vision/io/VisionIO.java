package team5427.frc.robot.subsystems.Vision.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    @AutoLog
    public class  VisionIOInputs {
        public Pose3d reportedPose = new Pose3d();
        public double ambiguity = 0.0;
        
        public double[] timestamps = new double[] {};
        public double[][] frames = new double[][] {};
        public long fps = 0;
        
    }


    public void updateInputs(VisionIOInputs inputs);

    public void setResetPose(Pose3d resetPose);
    
    public default void setPipeline(int pipelineNumber){}

    public void applyCameraTransformation(Pose3d transformation);

    public default void setStdDev(Matrix<N3, N1> stddev){}
}
