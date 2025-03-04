// package team5427.frc.robot;

// import static edu.wpi.first.units.Units.Meter;

// import org.opencv.core.Mat.Tuple4;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.LinearVelocity;
// import team5427.lib.detection.tuples.Tuple5Plus;
// import team5427.lib.kinematics.inverse.ArmInverseKinematics;
// import team5427.lib.kinematics.inverse.ArmInverseKinematics.VariableLengthArm;

// public class ScoringConfiguration {
//   private Pose3d driveTrainTargetPose;
//   private Translation3d endEffectorLocation;
//   private Rotation2d cascadeAngle;
//   private Distance cascadeLength;
//   private Rotation2d endEffectorAngle;

//   private boolean isVariable = false;

//   private ArmInverseKinematics generic_IK = new ArmInverseKinematics();

//   private VariableLengthArm ik;

//   public ScoringConfiguration(){
//     driveTrainTargetPose = new Pose3d();
//     cascadeAngle = Rotation2d.kZero;
//     cascadeLength = Meter.of(0.0);
//     endEffectorAngle = Rotation2d.kZero;
//     ik = generic_IK.new VariableLengthArm(ScoringConstants.kSegmentLengths, endEffectorAngle, endEffectorLocation);
//   }

//   public ScoringConfiguration(Pose3d driveTrainTargetPose, Rotation2d cascadeAngle, Distance casadeLength, Rotation2d endEffectorAngle){
//     this.driveTrainTargetPose = driveTrainTargetPose;
//     this.cascadeAngle = cascadeAngle;
//     this.cascadeLength = casadeLength;
//     this.endEffectorAngle = endEffectorAngle;
//     ik = generic_IK.new VariableLengthArm(ScoringConstants.kSegmentLengths, endEffectorAngle, endEffectorLocation);
//   }

//   public ScoringConfiguration(Pose3d driveTrainTargetPose, Rotation2d cascadeAngle, Distance casadeLength, Rotation2d endEffectorAngle, boolean isVariable){
//     this.driveTrainTargetPose = driveTrainTargetPose;
//     this.cascadeAngle = cascadeAngle;
//     this.cascadeLength = casadeLength;
//     this.endEffectorAngle = endEffectorAngle;
//     this.isVariable = isVariable;
//     ik = generic_IK.new VariableLengthArm(ScoringConstants.kSegmentLengths, endEffectorAngle, endEffectorLocation);
//   }

  

//   public Tuple5Plus<Rotation2d, Rotation2d, Distance, 


// }
