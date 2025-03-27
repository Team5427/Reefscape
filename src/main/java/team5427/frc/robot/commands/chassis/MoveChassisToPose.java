package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class MoveChassisToPose extends Command {

  private SwerveSubsystem swerveSubsystem;

  public MoveChassisToPose() {
    swerveSubsystem = SwerveSubsystem.getInstance();
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d targetPose =
        RobotState.getInstance()
            .getAdaptivePose()
            .nearest(List.of(RobotConfigConstants.kAlignPoses));

    swerveSubsystem.setInputSpeeds(swerveSubsystem.getDriveSpeeds(targetPose));
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
