package team5427.frc.robot.commands.chassis.assistance;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class BestAutoAlign extends Command {
  private ProfiledPIDController xController, yController, rotController;
  private boolean isRightScore;
  private SwerveSubsystem driveBase;
  public double tagID = -1;
  private static Pose2d targetPose;
  private static Pose2d robotPose2d;

  public BestAutoAlign(boolean isRightScore, SwerveSubsystem driveBase) {
    rotController =
        new ProfiledPIDController(
            Constants.SwerveConstants.kAutoAlignRotationalKp,
            0.0,
            0.0,
            new Constraints(10.0, 10.0));
    xController =
        new ProfiledPIDController(
            Constants.SwerveConstants.kAutoAlignTranslationKp,
            0.0,
            0.0,
            new Constraints(10.0, 10.0));
    yController =
        new ProfiledPIDController(
            Constants.SwerveConstants.kAutoAlignTranslationKp,
            0.0,
            0.0,
            new Constraints(10.0, 10.0));
    this.isRightScore = isRightScore;
    this.driveBase = driveBase;
    addRequirements(driveBase);
    if (DriverStation.isTeleop()) {
      List<Pose2d> actualPoses;
      List<Pose2d> targetPoses = new ArrayList<>();
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesBlue));
      } else {
        actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesRed));
      }
      for (int i = isRightScore ? 0 : 1; i < actualPoses.size(); i += 2) {
        targetPoses.add(actualPoses.get(i));
        System.out.println(actualPoses.get(i));
      }
      targetPose = RobotState.getInstance().getAdaptivePose().nearest(targetPoses);
    }
    robotPose2d = new Pose2d();
  }

  @Override
  public void initialize() {
    // List<Pose2d> actualPoses;
    // List<Pose2d> targetPoses = new ArrayList<>();

    // for (int i = isRight ? 0 : 1; i<actualPoses.size(); i+=2) {
    //     targetPoses.add(actualPoses.get(i));
    //     System.out.println(actualPoses.get(i));
    // }

    // rotController.setGoal(RobotState.getInstance().getAdaptivePose().nearest(targetPoses));
    List<Pose2d> actualPoses;
    List<Pose2d> targetPoses = new ArrayList<>();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesBlue));
    } else {
      actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesRed));
    }
    for (int i = isRightScore ? 0 : 1; i < actualPoses.size(); i += 2) {
      targetPoses.add(actualPoses.get(i));
      System.out.println(actualPoses.get(i));
    }

    // rotController.setGoal(rotController.calculate(
    //     Pose2d.kZero,
    //     targetPose.relativeTo(robotPose2d),
    //     SwerveConstants.kAutoAlignTranslationalMaxSpeed,
    //     targetPose.relativeTo(robotPose2d).getRotation()));
    rotController.setGoal(RobotState.getInstance().getEstimatedPose().getRotation().getRadians());
    rotController.setTolerance(0.05);

    xController.setGoal(RobotState.getInstance().getEstimatedPose().getX());
    xController.setTolerance(0.04);

    yController.setGoal(
        isRightScore
            ? RobotState.getInstance().getEstimatedPose().getY()
            : RobotState.getInstance().getEstimatedPose().getY() * -1);
    yController.setTolerance(0.04);
  }
}
