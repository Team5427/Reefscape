package team5427.frc.robot.subsystems.Vision.io.Quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class QuestCalibration {
  // -- Calculate Quest Offset (copied from
  // https://github.com/FRC5010/Reefscape2025/blob/main/TigerShark2025/src/main/java/org/frc5010/common/sensors/camera/QuestNav.java#L65) --

  private Translation2d calculatedOffsetToRobot = Translation2d.kZero;
  private double calculateOffsetCount = 1;

  private Translation2d calculateOffsetToRobot(Pose2d questRobotPose) {
    Rotation2d angle = questRobotPose.getRotation();
    Translation2d displacement = questRobotPose.getTranslation();

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / (2 * (1 - angle.getCos()));
    double y =
        ((-angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / (2 * (1 - angle.getCos()));

    return new Translation2d(x, y);
  }

  /**
   * When calibrating make sure the rotation in your quest transform is right. (Reality check) If it
   * is non-zero, you may have to swap the x/y and their signs.
   */
  public Command determineOffsetToRobotCenter(SwerveSubsystem swerve, Supplier<Pose2d> questPoseSupplier) {
    return Commands.repeatingSequence(
            Commands.run(
                    () -> {
                      swerve.setInputSpeeds(new ChassisSpeeds(0, 0, Math.PI / 10.0));
                    },
                    swerve)
                .withTimeout(0.5),
            Commands.runOnce(
                    () -> {
                      // Update current offset
                      Translation2d offset = calculateOffsetToRobot(questPoseSupplier.get());

                      calculatedOffsetToRobot =
                          calculatedOffsetToRobot
                              .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                              .plus(offset.div(calculateOffsetCount + 1));
                      calculateOffsetCount++;
                      Logger.recordOutput(
                          "QuestCalibration/CalculatedOffset", calculatedOffsetToRobot);
                    })
                .onlyIf(() -> questPoseSupplier.get().getRotation().getDegrees() > 30))
        .finallyDo(
            () -> {
              // Update current offset
              Translation2d offset = calculateOffsetToRobot(questPoseSupplier.get());

              calculatedOffsetToRobot =
                  calculatedOffsetToRobot
                      .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                      .plus(offset.div(calculateOffsetCount + 1));
              calculateOffsetCount++;
              Logger.recordOutput("QuestCalibration/CalculatedOffset", calculatedOffsetToRobot);
            });
  }
}