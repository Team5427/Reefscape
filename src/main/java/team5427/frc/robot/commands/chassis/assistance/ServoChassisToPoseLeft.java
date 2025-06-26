package team5427.frc.robot.commands.chassis.assistance;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * Aligns the Chassis to a Pose in the
 *
 * <pre>robot-relative {@code y} and {@code theta} </pre>
 *
 * This Command also allows for driver input to control
 *
 * <pre>{@code field-relative x}</pre>
 *
 * with a virtual heading zero of the pose's theta
 */
public class ServoChassisToPoseLeft extends Command {
  private SwerveSubsystem swerveSubsystem;
  private VisionSubsystem visionSubsystem;

  private CommandXboxController joy;
  private TunedJoystick translationJoystick;

  private MedianFilter xTargetFilter = new MedianFilter(5);

  private static Pose2d targetPose;

  public ServoChassisToPoseLeft(CommandXboxController driverJoystick) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    visionSubsystem = VisionSubsystem.getInstance();

    joy = driverJoystick;
    translationJoystick = new TunedJoystick(joy.getHID());
    translationJoystick.useResponseCurve(ResponseCurve.LINEAR);

    translationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    addRequirements(swerveSubsystem);

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
  }

  public static void setTargetPose(Pose2d pose2d) {
    targetPose = pose2d;
  }

  public ServoChassisToPoseLeft(CommandXboxController driverJoystick, Pose2d targetPose) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    visionSubsystem = VisionSubsystem.getInstance();

    joy = driverJoystick;
    translationJoystick = new TunedJoystick(joy.getHID());
    translationJoystick.useResponseCurve(ResponseCurve.LINEAR);

    translationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    addRequirements(swerveSubsystem);

    ServoChassisToPoseLeft.targetPose = targetPose;
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

    xTargetFilter.reset();
  }

  @Override
  public void execute() {

    Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
    Logger.recordOutput("Target Pose", targetPose);

    ChassisSpeeds adjustmentSpeeds =
        new ChassisSpeeds(
            0,
            SwerveConstants.kAutoAlignServoController.calculate(
                xTargetFilter.calculate(visionSubsystem.getTargetX(0).getDegrees()),
                SwerveConstants.kServoAprilTagTargetLeft.getDegrees()),
            SwerveConstants.kRotationPIDController.calculate(
                0.0, targetPose.getRotation().minus(robotPose.getRotation()).getRadians()));

    double vx = translationJoystick.getRightY();

    double dampener = (joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount);

    ChassisSpeeds driverSpeeds =
        swerveSubsystem.getDriveSpeeds(
            adjustmentSpeeds.vxMetersPerSecond,
            adjustmentSpeeds.vyMetersPerSecond,
            adjustmentSpeeds.omegaRadiansPerSecond,
            dampener);

    // establishes an aligned field relative speed for the x
    driverSpeeds.vxMetersPerSecond =
        swerveSubsystem.getDriveSpeeds(
                vx, 0.0, 0.0, dampener, targetPose.getRotation().plus(Rotation2d.k180deg))
            .vxMetersPerSecond;

    if (joy.getLeftTriggerAxis() >= 0.1) {
      driverSpeeds = new ChassisSpeeds(0, 0, 0);
    }
    swerveSubsystem.setInputSpeeds(driverSpeeds);
  }

  @Override
  public boolean isFinished() {

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
  }
}
