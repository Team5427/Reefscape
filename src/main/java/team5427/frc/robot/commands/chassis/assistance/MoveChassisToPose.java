package team5427.frc.robot.commands.chassis.assistance;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

/** Moves the Chassis to a Specified Pose in the robot-relative x, y, and theta parameters */
public class MoveChassisToPose extends Command {

  private SwerveSubsystem swerveSubsystem;
  private HolonomicDriveController driveController;

  private static Pose2d targetPose;

  public MoveChassisToPose() {
    swerveSubsystem = SwerveSubsystem.getInstance();
    addRequirements(swerveSubsystem);

    if (DriverStation.isTeleop()
        && DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {

      targetPose =
          RobotState.getInstance()
              .getAdaptivePose()
              .nearest(List.of(RobotConfigConstants.kAlignPosesRed));

    } else if (DriverStation.isTeleop() && Constants.kAlliance.get() == Alliance.Blue) {
      targetPose =
          RobotState.getInstance()
              .getAdaptivePose()
              .nearest(List.of(RobotConfigConstants.kAlignPosesBlue));
    }
  }

  public static void setTargetPose(Pose2d pose2d) {
    targetPose = pose2d;
  }

  public MoveChassisToPose(Pose2d targetPose) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    addRequirements(swerveSubsystem);

    MoveChassisToPose.targetPose = targetPose;
  }

  @Override
  public void initialize() {
    if (DriverStation.isTeleop() && Constants.kAlliance.get() == Alliance.Red) {

      targetPose =
          RobotState.getInstance()
              .getAdaptivePose()
              .nearest(List.of(RobotConfigConstants.kAlignPosesRed));

    } else if (DriverStation.isTeleop() && Constants.kAlliance.get() == Alliance.Blue) {
      targetPose =
          RobotState.getInstance()
              .getAdaptivePose()
              .nearest(List.of(RobotConfigConstants.kAlignPosesBlue));
    }

    driveController =
        new HolonomicDriveController(
            SwerveConstants.kTranslationXPIDController,
            SwerveConstants.kTranslationYPIDController,
            SwerveConstants.kRotationPIDController);
    driveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(2)));

    // targetPose = Constants.kAlliance.get() == Alliance.Red ?
    // FlippingUtil.flipFieldPose(targetPose): targetPose;

    // SwerveConstants.kTranslationPIDController.reset(
    //
    // RobotState.getInstance().getAdaptivePose().getTranslation().getDistance(targetPose.getTranslation())
    // );

    // driveController.setTolerance(targetPose.plus(new Transform2d(0.05, 0.05,
    // Rotation2d.fromDegrees(1))));

  }

  @Override
  public void execute() {

    Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
    Logger.recordOutput("Target Pose", targetPose);

    ChassisSpeeds adjustmentSpeeds =
        driveController.calculate(
            Pose2d.kZero,
            targetPose.relativeTo(robotPose),
            SwerveConstants.kAutoAlignTranslationalMaxSpeed,
            targetPose.getRotation().minus(robotPose.getRotation()));

    if (driveController.atReference()) {
      swerveSubsystem.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
      // return true;
    } else {
      swerveSubsystem.setInputSpeeds(adjustmentSpeeds);
    }
  }

  @Override
  public boolean isFinished() {

    return driveController.atReference();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
  }
}
