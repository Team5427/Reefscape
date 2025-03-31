package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.List;
import java.util.Optional;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;

import team5427.frc.robot.RobotState;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ChassisMovement extends Command {

  private SwerveSubsystem swerveSubsystem;
  private CommandXboxController joy;
  private Optional<CommandXboxController> simulatedRotationalJoy;

  private TunedJoystick translationJoystick;
  private TunedJoystick rotationJoystick;

  public ChassisMovement(CommandXboxController driverJoystick) {
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
      double vx = -translationJoystick.getRightY();
      double vy = -translationJoystick.getRightX();
      double omegaRadians = -rotationJoystick.getLeftX() * Math.abs(translationJoystick.getLeftX());

      double dampener = (joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount);

      ChassisSpeeds driverSpeeds = swerveSubsystem.getDriveSpeeds(vx, vy, omegaRadians, dampener);

      if (joy.getLeftTriggerAxis() >= 0.1) {
        driverSpeeds = new ChassisSpeeds(0, 0, 0);
      }
      swerveSubsystem.setInputSpeeds(driverSpeeds);

      // Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
      // if (robotPose.getTranslation().getDistance(robotPose.nearest(List.of(RobotConfigConstants.kAlignPoses)).getTranslation()) < Units.inchesToMeters(100.0)) {
      //   joy.setRumble(RumbleType.kBothRumble, 0.5);
      //   System.out.println("Near Goal");
      //   // altJoy.setRumble(RumbleType.kBothRumble, 0.1);
      // } else {
      //   joy.setRumble(RumbleType.kBothRumble, 0);
      //   // altJoy.setRumble(RumbleType.kBothRumble, 0);
      // }
    } else {
      swerveSubsystem.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
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
