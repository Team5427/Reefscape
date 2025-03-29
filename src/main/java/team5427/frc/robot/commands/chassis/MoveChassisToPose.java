package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.Constants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class MoveChassisToPose extends Command {

  private SwerveSubsystem swerveSubsystem;
  private HolonomicDriveController driveController;

  private Pose2d targetPose;
  private boolean lazyControl;

  public MoveChassisToPose(boolean lazyControl) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {

    targetPose =
        RobotState.getInstance()
            .getAdaptivePose()
            .nearest(List.of(RobotConfigConstants.kAlignPoses));

    // SwerveConstants.kTranslationPIDController.reset(
    //   RobotState.getInstance().getAdaptivePose().getTranslation().getDistance(targetPose.getTranslation())
    // );

    driveController = new HolonomicDriveController(
      SwerveConstants.kTranslationXPIDController, 
      SwerveConstants.kTranslationYPIDController, 
      SwerveConstants.kRotationPIDController
    );
    // driveController.setTolerance(targetPose.plus(new Transform2d(0.05, 0.05, Rotation2d.fromDegrees(1))));
    driveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(2)));
  }

  @Override
  public void execute() {

    Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
    Logger.recordOutput("Target Pose", targetPose);

    if (lazyControl) {
      
      double inputX = SwerveConstants.kTranslationXPIDController.calculate(robotPose.getX(), targetPose.getX());
      double inputY = SwerveConstants.kTranslationYPIDController.calculate(robotPose.getY(), targetPose.getY());
      double calculatedOmega = SwerveConstants.kRotationPIDController.calculate(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

      ChassisSpeeds translationalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(inputX, inputY, 0.0), swerveSubsystem.getGyroRotation());
      ChassisSpeeds rotationalSpeeds = new ChassisSpeeds(0.0, 0.0, calculatedOmega);

      ChassisSpeeds adjustmentSpeeds = translationalSpeeds.plus(rotationalSpeeds);
      swerveSubsystem.setInputSpeeds(adjustmentSpeeds);

    } else {

      ChassisSpeeds adjustmentSpeeds = driveController.calculate(Pose2d.kZero, targetPose.relativeTo(robotPose), SwerveConstants.kDriveMotorConfiguration.maxVelocity * 0.1, targetPose.getRotation());
      swerveSubsystem.setInputSpeeds(adjustmentSpeeds);

    }
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
