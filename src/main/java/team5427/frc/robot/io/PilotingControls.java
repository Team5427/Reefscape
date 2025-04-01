package team5427.frc.robot.io;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.commands.chassis.ChassisMovement;
import team5427.frc.robot.commands.chassis.LockedChassisMovement;
import team5427.frc.robot.commands.chassis.MoveChassisToPose;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class PilotingControls {

  private CommandXboxController joy;

  public static final Trigger rumble =
      new Trigger(
          () ->
              RobotState.getInstance()
                      .getAdaptivePose()
                      .getTranslation()
                      .getDistance(
                          RobotState.getInstance()
                              .getAdaptivePose()
                              .nearest(List.of(RobotConfigConstants.kAlignPoses))
                              .getTranslation())
                  < Units.inchesToMeters(1.5));

  public PilotingControls() {

    joy = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // joy.a()
    //     .onTrue(new CoralIntakeTest())
    //     .whileFalse(
    //         new InstantCommand(
    //             () -> {
    //               EndEffectorSubsystem.state = EndEffectorStates.IDLE;
    //               EndEffectorSubsystem.getInstance()
    //                   .setAlgaeRollerSetpoint(MetersPerSecond.of(0.0));
    //               EndEffectorSubsystem.getInstance()
    //                   .setCoralRollerSetpoint(MetersPerSecond.of(0.0));
    //               EndEffectorSubsystem.getInstance()
    //                   .setPivotSetpoint(Rotation2d.fromDegrees(-10.0));

    // EndEffectorSubsystem.getInstance().setWristSetpoint(Rotation2d.fromDegrees(0.0));
    //             },
    //             EndEffectorSubsystem.getInstance()));

    // joy.b()
    //     .whileTrue(new CascadeTest())
    //     .whileFalse(
    //         new InstantCommand(
    //             () -> {
    //               //
    // CascadeSubsystem.getInstance().setCascadeEncoderPosition(Distance.ofBaseUnits(0.0,
    //               // Meters));
    //               CascadeSubsystem.getInstance()
    //                   .setCascadeSetpoint(Distance.ofRelativeUnits(.5, Meters));
    //               CascadeSubsystem.getInstance().setPivotSetpoint(Rotation2d.fromDegrees(0.1));
    //             }));
    Pose2d blueResetPose = new Pose2d(5.76, 4.022, Rotation2d.kZero);
    Pose2d redResetPose = FlippingUtil.flipFieldPose(blueResetPose);

    Logger.recordOutput("Red Pose", redResetPose);
    Logger.recordOutput("Blue Pose", blueResetPose);

    joy.b()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotState.getInstance()
                      .resetAllPose(
                          DriverStation.getAlliance().get() == Alliance.Red
                              ? redResetPose
                              : blueResetPose,
                          SwerveSubsystem.getInstance().getModulePositions(),
                          SwerveSubsystem.getInstance().getGyroRotation());
                }));
    // AutoBuilder.resetOdom(DriverStation.getAlliance().get() == Alliance.Red ? redResetPose:
    // blueResetPose).ignoringDisable(true));

    SwerveSubsystem.getInstance().setDefaultCommand(new ChassisMovement(joy));

    // joy.leftBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               SwerveSubsystem.getInstance().setGyroLock(true);
    //             }))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               SwerveSubsystem.getInstance().setGyroLock(false);
    //             }));

    joy.leftBumper().whileTrue(new LockedChassisMovement(joy));
    joy.a()
        .onTrue(
            SwerveSubsystem.getInstance()
                .followPosePathFinding(RobotState.getInstance().getClosestReefPose()));
    // joy.a().whileTrue(new MoveChassisToPose(true));
    joy.x().whileTrue(new MoveChassisToPose(false));

    // joy.a().onTrue(AutoBuilder.followPath(
    //   new PathPlannerPath(
    //     PathPlannerPath.waypointsFromPoses(
    //       List.of(
    //         RobotState.getInstance().getAdaptivePose(),
    //
    // RobotState.getInstance().getAdaptivePose().nearest(List.of(RobotConfigConstants.kReefPoses))
    //       )),
    //     new PathConstraints(
    //       MetersPerSecond.of(SwerveConstants.kDriveMotorConfiguration.maxVelocity * 0.25),
    //       MetersPerSecondPerSecond.of(SwerveConstants.kDriveMotorConfiguration.maxAcceleration),
    //       RotationsPerSecond.of(Math.PI * 0.25),
    //       RotationsPerSecondPerSecond.of(Math.PI)),
    //     null,
    //     new GoalEndState(0.0,
    // RobotState.getInstance().getAdaptivePose().nearest(List.of(RobotConfigConstants.kReefPoses)).getRotation()))
    // ));

    // joy.a()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new InstantCommand(
    //                 () -> {
    //                   RobotState.getInstance()
    //                       .resetAllPose(
    //
    // VisionSubsystem.getInstance().getLatestPoseMeasurement().toPose2d());
    //                 }),
    //             new InstantCommand(),
    //             () -> {
    //               return VisionSubsystem.getInstance().getLatestPoseMeasurement() !=
    // Optional.empty();
    //             }));
    joy.y()
        .and(() -> RobotBase.isReal())
        .onTrue(
            new InstantCommand(
                    () -> {
                      SwerveSubsystem.getInstance().resetGyro(Rotation2d.kZero);
                      RobotState.getInstance().resetHeading(Rotation2d.kZero);
                    })
                .ignoringDisable(true));

    rumble.onTrue(
        new InstantCommand(
            () -> {
              joy.setRumble(RumbleType.kBothRumble, 0.1);
            }));
    rumble.onFalse(
        new InstantCommand(
            () -> {
              joy.setRumble(RumbleType.kBothRumble, 0);
            }));

    // SwerveSubsystem.getInstance().setPose(VisionSubsystem.getInstance().getLatestPose().toPose2d());
  }
}
