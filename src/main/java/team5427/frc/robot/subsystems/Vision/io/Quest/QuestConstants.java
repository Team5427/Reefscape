package team5427.frc.robot.subsystems.Vision.io.Quest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class QuestConstants {
  public static final Transform2d robotToQuestTransform =
      new Transform2d(
          Units.inchesToMeters(2.274634152842048),
          Units.inchesToMeters(7.481121673003204),
          Rotation2d.kCCW_90deg);

  public static final Matrix<N3, N1> stdDevs = VecBuilder.fill(0, 0, 0);
}
