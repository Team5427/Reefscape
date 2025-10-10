import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import team5427.frc.robot.Constants.RobotConfigConstants;

public class AutoAlignTest {
  public AutoAlignTest() {}

  @Test
  public void autoAlignTest() {
    Pose2d robotPose = new Pose2d(2, 2, new Rotation2d());
    Pose2d targetPose;
    List<Pose2d> actualPoses;
    List<Pose2d> targetPoses = new ArrayList<>();
    actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesBlue));
    for (int i = 0; i < actualPoses.size() - 1; i += 2) {
      targetPoses.add(actualPoses.get(i));
      System.out.println(actualPoses.get(i));
    }
    targetPose = robotPose.nearest(targetPoses);
    System.out.println(List.of(targetPoses));
    System.out.println(targetPoses.size());
    System.out.println("Target Pose: " + targetPose);
    System.out.println(robotPose.relativeTo(targetPose));
    System.out.println("break");
  }

  @Test
  public void autoAlignCommandTest() {
    // replace that dead code conditionals with different ones to mock different behaviors
    List<Pose2d> actualPoses;
    List<Pose2d> targetPoses = new ArrayList<>();
    if (false) {
      actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesBlue));
    } else {
      actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesRed));
    }
    for (int i = false ? 0 : 1; i < actualPoses.size(); i += 2) {
      targetPoses.add(actualPoses.get(i));
      System.out.println(actualPoses.get(i));
    }
    System.out.println(
        "Target Pose: "
            + new Pose2d(13.88, 5.12, Rotation2d.kZero).nearest(targetPoses).toString());
  }
}
