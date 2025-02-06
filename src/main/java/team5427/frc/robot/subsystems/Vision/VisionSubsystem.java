package team5427.frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.VisionConstants;
import team5427.frc.robot.subsystems.Vision.io.VisionIO;
import team5427.frc.robot.subsystems.Vision.io.VisionIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Vision.io.VisionIOPhoton;

public class VisionSubsystem extends SubsystemBase{
    private VisionIO io;
    private VisionIOInputsAutoLogged inputsAutoLogged;
    private Pose3d referencePose;

    public VisionSubsystem(){
        switch (Constants.currentMode) {
            case REAL:
                io = new VisionIOPhoton(VisionConstants.swerveCamName);
                break;
        
            default:
                break;
        }
    }
}
