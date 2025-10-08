package team5427.frc.robot.commands.chassis.assistance;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AlignChassisFromPolar extends Command {
  private SwerveSubsystem swerve;
  private ProfiledPIDController distanceController;
  private ProfiledPIDController angleController;
  private TunedJoystick translationJoystick;
  private CommandXboxController joy;
  private boolean isRight;

  private static Pose2d targetPose;

  public AlignChassisFromPolar(CommandXboxController driverJoystick, boolean isRight) {
    swerve = SwerveSubsystem.getInstance();
    this.joy = driverJoystick;

    distanceController = SwerveConstants.kTranslationPIDController;
    distanceController.setTolerance(0.02);

    angleController = SwerveConstants.kRotationPIDController;

    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Math.toRadians(1));

    translationJoystick = new TunedJoystick(driverJoystick.getHID());
    translationJoystick.useResponseCurve(ResponseCurve.LINEAR);
    translationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    addRequirements(swerve);
    this.isRight = isRight;

    if (DriverStation.isTeleop()) {
      List<Pose2d> actualPoses;
      List<Pose2d> targetPoses = new ArrayList<>();
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Blue) {
        actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesBlue));
      } else {
        actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesRed));
      }
      for (int i = isRight ? 0 : 1; i < actualPoses.size(); i += 2) {
        targetPoses.add(actualPoses.get(i));
        System.out.println(actualPoses.get(i));
      }
      targetPose = RobotState.getInstance().getAdaptivePose().nearest(targetPoses);
    }
  }

  @Override
  public void initialize() {
    distanceController.reset(0);
    distanceController.setTolerance(0.05);
    angleController.reset(0);
    angleController.setTolerance(Units.degreesToRadians(7));
    if (DriverStation.isTeleop()) {
      List<Pose2d> actualPoses;
      List<Pose2d> targetPoses = new ArrayList<>();
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesBlue));
      } else {
        actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesRed));
      }
      for (int i = isRight ? 0 : 1; i < actualPoses.size(); i += 2) {
        targetPoses.add(actualPoses.get(i));
        System.out.println(actualPoses.get(i));
      }
      targetPose = RobotState.getInstance().getAdaptivePose().nearest(targetPoses);
    }
  }

  @Override
  public void execute() {
    Logger.recordOutput("Target Pose", targetPose);
    Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
    Logger.recordOutput("Relative Target Pose", targetPose.relativeTo(robotPose));
    Pose2d robotRelativePose = targetPose.relativeTo(robotPose);

    double omegaRadiansPerSecond =
        angleController.calculate(robotRelativePose.getRotation().getRadians() + Math.PI, 0);
    double alignmentSpeed =
        distanceController.calculate(robotRelativePose.getTranslation().getNorm(), 0);

    double vx = -alignmentSpeed * robotRelativePose.getRotation().getCos();
    double vy = -alignmentSpeed * robotRelativePose.getRotation().getSin();

    double dampener = (joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount);
    ChassisSpeeds fieldRelativeSpeeds =
        swerve.getDriveSpeeds(vx, vy, omegaRadiansPerSecond, dampener, robotPose.getRotation().unaryMinus());
    // fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotPose.getRotation());
    if (joy.getLeftTriggerAxis() >= 0.1) {
      fieldRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
    }
    swerve.setInputSpeeds(fieldRelativeSpeeds);
  }

  @Override
  public boolean isFinished() {
    return targetPose
                .getTranslation()
                .minus(RobotState.getInstance().getAdaptivePose().getTranslation())
                .getNorm()
            < 0.02
        && angleController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
  }
}
