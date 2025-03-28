package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class MoveChassisToPose extends Command {

  private SwerveSubsystem swerveSubsystem;

  private Pose2d targetPose;

  public MoveChassisToPose() {
    swerveSubsystem = SwerveSubsystem.getInstance();
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {

    targetPose =
        RobotState.getInstance()
            .getAdaptivePose()
            .nearest(List.of(RobotConfigConstants.kAlignPoses));

    SwerveConstants.kTranslationPIDController.reset(
        RobotState.getInstance()
            .getAdaptivePose()
            .getTranslation()
            .getDistance(targetPose.getTranslation()));
  }

  @Override
  public void execute() {

    Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
    stupidway();

    // double dx = targetPose.getX() - robotPose.getX();
    // double dy = targetPose.getY() - robotPose.getY();
    // double dist = Math.hypot(dx, dy);

    // Rotation2d fieldTheta = Rotation2d.fromRadians(Math.atan(dy/dx));
    // Rotation2d robotTheta = fieldTheta.minus(swerveSubsystem.getGyroRotation());

    // double desiredX = dist * robotTheta.getCos();
    // double desiredY = dist * robotTheta.getSin();

    // double speedsX = SwerveConstants.kTranslationXPIDController.calculate(0, dx);

    // double speedsX = SwerveConstants.kTranslationXPIDController.calculate(robotPose.getX(),
    // targetPose.getX());
    // double speedsY = SwerveConstants.kTranslationYPIDController.calculate(robotPose.getY(),
    // targetPose.getY());
    // double calculatedOmega =
    //     SwerveConstants.kRotationPIDController.calculate(
    //         RobotState.getInstance().getAdaptivePose().getRotation().getRadians(),
    //         targetPose.getRotation().getRadians());

    // swerveSubsystem.setInputSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new
    // ChassisSpeeds(speedsX, speedsY, calculatedOmega), swerveSubsystem.getGyroRotation()));
  }

  public void stupidway() {
    Pose2d robotPose = RobotState.getInstance().getAdaptivePose();

    double distanceError = robotPose.getTranslation().getDistance(targetPose.getTranslation());
    if (robotPose.getTranslation().getNorm() > targetPose.getTranslation().getNorm()) {
      distanceError *= -1;
    }

    double distanceVelocitySetpoint =
        SwerveConstants.kTranslationPIDController.calculate(distanceError, 0);

    // in radians per second
    double thetaVelocitySetpoint =
        SwerveConstants.kRotationPIDController.calculate(
            robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    Translation2d robotMovement =
        new Translation2d(distanceVelocitySetpoint, (robotPose.getRotation()));
    Pose2d robotMovementPose = new Pose2d(robotMovement, robotPose.getRotation());
    Logger.recordOutput("Robot Movement", robotMovementPose);
    swerveSubsystem.setInputSpeeds(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            -robotMovement.getX(),
            robotMovement.getY(),
            thetaVelocitySetpoint,
            robotPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setInputSpeeds(new ChassisSpeeds());
  }
}
