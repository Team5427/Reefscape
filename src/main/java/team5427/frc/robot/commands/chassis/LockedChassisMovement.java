package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import java.util.Optional;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class LockedChassisMovement extends Command {

  private SwerveSubsystem swerveSubsystem;
  private CommandXboxController joy;
  private Optional<CommandXboxController> simulatedRotationalJoy;

  private TunedJoystick tunedJoystickLinear;
  private TunedJoystick tunedJoystickQuadratic;

  private Rotation2d rotationSetpoint = new Rotation2d();

  private Pose2d[] matchingPoses;

  private Pose2d robotPose = new Pose2d();

  public LockedChassisMovement(CommandXboxController driverJoystick, Rotation2d rotationSetpoint) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    joy = driverJoystick;
    tunedJoystickLinear = new TunedJoystick(joy.getHID());
    tunedJoystickLinear.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickQuadratic = new TunedJoystick(joy.getHID());
    tunedJoystickQuadratic.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickLinear.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    tunedJoystickQuadratic.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    this.rotationSetpoint = rotationSetpoint;

    addRequirements(swerveSubsystem);
  }

  public LockedChassisMovement(CommandXboxController driverJoystick, Pose2d pose) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    joy = driverJoystick;
    tunedJoystickLinear = new TunedJoystick(joy.getHID());
    tunedJoystickLinear.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickQuadratic = new TunedJoystick(joy.getHID());
    tunedJoystickQuadratic.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickLinear.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    tunedJoystickQuadratic.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    this.rotationSetpoint = pose.getRotation();
    this.robotPose = swerveSubsystem.getPose();

    addRequirements(swerveSubsystem);
  }

  public LockedChassisMovement(
      CommandXboxController driverJoystick, Pose2d robotPose, Pose2d[] matchingPoses) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    joy = driverJoystick;
    tunedJoystickLinear = new TunedJoystick(joy.getHID());
    tunedJoystickLinear.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickQuadratic = new TunedJoystick(joy.getHID());
    tunedJoystickQuadratic.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickLinear.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    tunedJoystickQuadratic.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    this.matchingPoses = matchingPoses;
    this.robotPose = robotPose;

    addRequirements(swerveSubsystem);
  }

  public LockedChassisMovement(
      CommandXboxController driverJoystick, Pose2d robotPose, List<Pose2d> matchingPoses) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    joy = driverJoystick;
    tunedJoystickLinear = new TunedJoystick(joy.getHID());
    tunedJoystickLinear.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickQuadratic = new TunedJoystick(joy.getHID());
    tunedJoystickQuadratic.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickLinear.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    tunedJoystickQuadratic.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    this.matchingPoses = (Pose2d[]) matchingPoses.toArray();
    this.robotPose = robotPose;

    addRequirements(swerveSubsystem);
  }

  public LockedChassisMovement(CommandXboxController driverJoystick, Pose2d[] matchingPoses) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    joy = driverJoystick;
    tunedJoystickLinear = new TunedJoystick(joy.getHID());
    tunedJoystickLinear.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickQuadratic = new TunedJoystick(joy.getHID());
    tunedJoystickQuadratic.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickLinear.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    tunedJoystickQuadratic.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    this.matchingPoses = matchingPoses;
    this.robotPose = swerveSubsystem.getPose();
    updateRotationSetpoint();
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.setFieldOp(DriverStation.getAlliance().get().equals(Alliance.Red));
    // SwerveConstants.kRotationPIDController.setConstraints(new Constraints(, 0));
  }

  @Override
  public void execute() {

    if (DriverStation.isTeleop()) {

      robotPose = swerveSubsystem.getPose();
      updateRotationSetpoint();
      double dampener = joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount;

      double vx = 0.0, vy = 0.0;
      vx = -tunedJoystickLinear.getRightY() * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
      vy = -tunedJoystickLinear.getRightX() * SwerveConstants.kDriveMotorConfiguration.maxVelocity;

      double omegaRadians =
          SwerveConstants.kRotationPIDController.calculate(
              SwerveSubsystem.getInstance().getGyroRotation().getRadians(),
              this.rotationSetpoint.getRadians());

      vx *= (1 - dampener);
      vy *= (1 - dampener);
      omegaRadians *= 0.1;
      omegaRadians *= (1 - dampener);

      ChassisSpeeds inputSpeeds = new ChassisSpeeds(vx, vy, omegaRadians);
      if (joy.getLeftTriggerAxis() >= 0.1) {
        inputSpeeds = new ChassisSpeeds(0, 0, 0);
        swerveSubsystem.stop(true);
      } else {
        swerveSubsystem.stop(false);
      }
      swerveSubsystem.setChassisSpeeds(inputSpeeds);
    }
  }

  public void updateRotationSetpoint() {
    this.rotationSetpoint =
        robotPose.nearest(List.of(matchingPoses)).getRotation().rotateBy(Rotation2d.k180deg);
  }

  public Rotation2d getRotationSetpoint() {
    return this.rotationSetpoint;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
