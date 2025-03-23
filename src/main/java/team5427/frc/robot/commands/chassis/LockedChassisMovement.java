package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.Field.Reef;
import team5427.frc.robot.Field;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class LockedChassisMovement extends Command {

  private SwerveSubsystem swerveSubsystem;
  private CommandXboxController joy;
  private Optional<CommandXboxController> simulatedRotationalJoy;

  private TunedJoystick translationJoystick;
  private TunedJoystick rotationJoystick;

  public LockedChassisMovement(CommandXboxController driverJoystick) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    joy = driverJoystick;
    translationJoystick = new TunedJoystick(joy.getHID());
    translationJoystick.useResponseCurve(ResponseCurve.LINEAR);

    rotationJoystick = new TunedJoystick(joy.getHID());
    rotationJoystick.useResponseCurve(ResponseCurve.LINEAR);

    translationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    rotationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    // swerveSubsystem.setFieldOp(DriverStation.getAlliance().get().equals(Alliance.Red));
  }

  @Override
  public void execute() {

    if (DriverStation.isTeleop()) {

      Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
      Rotation2d lockedSetpoint = robotPose.nearest(List.of(RobotConfigConstants.kReefPoses)).getRotation().rotateBy(Rotation2d.k180deg);

      double vx = -translationJoystick.getRightY();
      double vy = -translationJoystick.getRightX();

      double dampener = (joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount);
      ChassisSpeeds driverSpeeds = swerveSubsystem.getDriveSpeeds(vx, vy, lockedSetpoint, dampener);

      if(joy.getLeftTriggerAxis() > 0.1){
        driverSpeeds = new ChassisSpeeds(0,0,0);
      }
      
      
      swerveSubsystem.setInputSpeeds(driverSpeeds);
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