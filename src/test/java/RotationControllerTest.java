// import static org.junit.jupiter.api.Assertions.assertTrue;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import java.util.List;
// import org.junit.jupiter.api.Test;
// import team5427.frc.robot.Constants.RobotConfigConstants;
// import team5427.frc.robot.RobotState;
// import team5427.frc.robot.commands.chassis.LockedChassisMovement;

// public class RotationControllerTest {
//   public RotationControllerTest() {}

//   @Test
//   public void rotationControllerTest() {
//     CommandXboxController controller = new CommandXboxController(0);
//     RobotState.getInstance().resetAllPose(new Pose2d(1, 0.83, Rotation2d.kZero));
//     LockedChassisMovement lockedChassisMovement =
//         new LockedChassisMovement(controller, RobotConfigConstants.kReefPoses);
//     System.out.println("Rotation setpoint: " + lockedChassisMovement.getRotationSetpoint());
//     System.out.println("Robot Pose: " + RobotState.getInstance().getEstimatedPose());
//     Pose2d closestReefPose =
//         RobotState.getInstance()
//             .getEstimatedPose()
//             .nearest(List.of(RobotConfigConstants.kReefPoses));
//     System.out.println("Closest Reef Branch Pose: " + closestReefPose);
//     assertTrue(
//         closestReefPose
//             .getRotation()
//             .equals(lockedChassisMovement.getRotationSetpoint().rotateBy(Rotation2d.k180deg)));
//   }
// }
