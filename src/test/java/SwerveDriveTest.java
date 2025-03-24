import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveDriveTest {
  SwerveSubsystem swerveSubsystem;

  public SwerveDriveTest() {
    swerveSubsystem = SwerveSubsystem.getInstance();
  }

  @BeforeEach
  public void setup() {
    System.out.println(swerveSubsystem.getModuleStates());
  }

  @Test
  public void odometryTestPose() {
    Pose2d initialPose = RobotState.getInstance().getEstimatedPose();
    Pose2d resetPose = new Pose2d(3.4, 5, Rotation2d.fromDegrees(90));
    RobotState.getInstance().resetAllPose(resetPose);
    Pose2d afterPose = RobotState.getInstance().getEstimatedPose();
    assert afterPose.equals(resetPose);
  }

  @Test
  public void odometryTestRotation() {
    Pose2d initialPose = RobotState.getInstance().getEstimatedPose();
    Pose2d resetPose = new Pose2d(3.4, 5, Rotation2d.fromDegrees(90));
    RobotState.getInstance().resetAllPose(resetPose);
    Rotation2d afterPose = RobotState.getInstance().getEstimatedPose().getRotation();
    System.out.println("Initial Pose: " + initialPose);
    System.out.println("Reset Pose: " + resetPose);
    System.out.println("After Pose: " + afterPose);
    assert afterPose.equals(resetPose.getRotation());
  }

  @Test
  public void odometryTestWithVision() {
    Pose2d initialPose = RobotState.getInstance().getEstimatedPose();
    Pose2d odometryPose = new Pose2d(3.4, 5, Rotation2d.fromDegrees(90));
    RobotState.getInstance().resetAllPose(odometryPose);

    Pose2d newVisionPose = new Pose2d(4, 4, Rotation2d.fromDegrees(0));
    RobotState.getInstance()
        .addVisionMeasurement(newVisionPose, 0, VecBuilder.fill(0.01, 0.01, 0.8));
    Pose2d afterPose = RobotState.getInstance().getEstimatedPose();
    System.out.println("After Pose " + afterPose);
    System.out.println("Vision Pose Input " + newVisionPose);
    System.out.println("Odometry Pose " + odometryPose);
  }
}
