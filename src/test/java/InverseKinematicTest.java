import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;
import team5427.frc.robot.RobotContainer;
import team5427.lib.kinematics.inverse.ArmInverseKinematics;
import team5427.lib.kinematics.inverse.ArmInverseKinematics.VariableLengthArm;

public class InverseKinematicTest {

  public InverseKinematicTest() {
    // createRobotContainer();
    // inverseKinematicsTest();

  }

  @Test
  public void createRobotContainer() {
    // Instantiate RobotContainer
    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // Working
  @Test
  public void inverseKinematicsTest() {
    ArmInverseKinematics pIk = new ArmInverseKinematics();
    VariableLengthArm ik =
        pIk
        .new VariableLengthArm(
            new double[] {5}, Rotation2d.fromDegrees(80), new Translation3d(10, 0, 10));
    System.out.println(ik.getAngleAndArmLength().toString());
  }
}
