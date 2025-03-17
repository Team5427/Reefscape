package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ChassisMovement extends Command {

  private SwerveSubsystem swerveSubsystem;
  private CommandXboxController joy;
  private Optional<CommandXboxController> simulatedRotationalJoy;

  private TunedJoystick tunedJoystickLinear;
  private TunedJoystick tunedJoystickQuadratic;

  public ChassisMovement(CommandXboxController driverJoystick) {
    swerveSubsystem = SwerveSubsystem.getInstance(Optional.empty());
    joy = driverJoystick;
    tunedJoystickLinear = new TunedJoystick(joy.getHID());
    tunedJoystickLinear.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickQuadratic = new TunedJoystick(joy.getHID());
    tunedJoystickQuadratic.useResponseCurve(ResponseCurve.LINEAR);

    tunedJoystickLinear.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    tunedJoystickQuadratic.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.setFieldOp(DriverStation.getAlliance().get().equals(Alliance.Red));
  }

  @Override
  public void execute() {
    if (false && RobotBase.isSimulation()) {
      double vx = -joy.getRawAxis(0) * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
      double vy = -joy.getRawAxis(1) * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
      double omegaRadians = -simulatedRotationalJoy.get().getRawAxis(0) * Math.PI * 4.0;
      ChassisSpeeds inputSpeeds = new ChassisSpeeds(vx, vy, omegaRadians);
      swerveSubsystem.setChassisSpeeds(inputSpeeds);
      Logger.recordOutput("InputSpeeds", inputSpeeds);
    } else {

      if (DriverStation.isTeleop()) {

        double dampener = joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount;
        double vx = 0.0, vy = 0.0, omegaRadians = 0.0;
        vx =
            -tunedJoystickLinear.getRightY() * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
        vy =
            -tunedJoystickLinear.getRightX() * SwerveConstants.kDriveMotorConfiguration.maxVelocity;

        omegaRadians =
            -tunedJoystickQuadratic.getLeftX()
                * Math.abs(tunedJoystickLinear.getLeftX())
                * Math.PI
                * SwerveConstants.kDriveMotorConfiguration.maxVelocity;


        ChassisSpeeds inputSpeeds = new ChassisSpeeds(vx, vy, omegaRadians);
        inputSpeeds.times(MathUtil.clamp(1 - dampener, 0, 1));
        if (joy.getLeftTriggerAxis() >= 0.1) {
          inputSpeeds = new ChassisSpeeds(0, 0, 0);
          swerveSubsystem.setStopped(true);
          return;
        } else {
          swerveSubsystem.setStopped(false);
          swerveSubsystem.setChassisSpeeds(inputSpeeds);
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
