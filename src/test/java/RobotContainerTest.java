import org.junit.jupiter.api.Test;
import team5427.frc.robot.RobotContainer;

public class RobotContainerTest {
  @Test
  public void createRobotContainer() {
    // Instantiate RobotContainer

    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
