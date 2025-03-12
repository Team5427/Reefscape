import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
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
  public void odometryTest() {
    Pose2d initialPose = swerveSubsystem.getPose();
    Pose2d resetPose = new Pose2d(3.4, 5, Rotation2d.fromDegrees(90));
    swerveSubsystem.setPose(resetPose);
    Pose2d afterPose = swerveSubsystem.getPose();
    assert afterPose.equals(resetPose);
  }
}
