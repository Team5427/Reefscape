// import static org.junit.jupiter.api.Assertions.assertTrue;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import java.util.List;
// import org.junit.jupiter.api.Test;
// import team5427.frc.robot.Constants.RobotConfigConstants;
// import team5427.frc.robot.commands.chassis.LockedChassisMovement;
// import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

// public class RotationControllerTest {
//   public RotationControllerTest() {}

//   @Test
//   public void rotationControllerTest() {
//     CommandXboxController controller = new CommandXboxController(0);
//     SwerveSubsystem.getInstance().resetAutonPose(new Pose2d(1, 0.83, Rotation2d.kZero));
//     LockedChassisMovement lockedChassisMovement =
//         new LockedChassisMovement(controller, RobotConfigConstants.kReefPoses);
//     System.out.println("Rotation setpoint: " + lockedChassisMovement.getRotationSetpoint());
//     System.out.println("Robot Pose: " + SwerveSubsystem.getInstance().getPose());
//     Pose2d closestReefPose =
//         SwerveSubsystem.getInstance().getPose().nearest(List.of(RobotConfigConstants.kReefPoses));
//     System.out.println("Closest Reef Branch Pose: " + closestReefPose);
//     assertTrue(
//         closestReefPose
//             .getRotation()
//             .equals(lockedChassisMovement.getRotationSetpoint().rotateBy(Rotation2d.k180deg)));
//   }
// }
