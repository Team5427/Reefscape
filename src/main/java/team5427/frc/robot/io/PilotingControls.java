package team5427.frc.robot.io;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.commands.chassis.ChassisMovement;
import team5427.frc.robot.commands.chassis.assistance.AlignChassisFromPolar;
import team5427.frc.robot.commands.chassis.assistance.AlignChassisToCenter;
import team5427.frc.robot.commands.chassis.assistance.AlignChassisToSide;
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
                              .nearest(
                                  (DriverStation.getAlliance().isPresent()
                                          && DriverStation.getAlliance().get() == Alliance.Red)
                                      ? List.of(RobotConfigConstants.kAlignPosesRed)
                                      : List.of(RobotConfigConstants.kAlignPosesBlue))
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
    // Pose2d blueResetPose = new Pose2d(5.76, 4.022, Rotation2d.kZero);
    Pose2d blueResetPose = new Pose2d(3.2, 4.03, Rotation2d.kZero);
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
                    })
                .ignoringDisable(true));
    // AutoBuilder.resetOdom(DriverStation.getAlliance().get() == Alliance.Red ? redResetPose:
    // blueResetPose).ignoringDisable(true));

    SwerveSubsystem.getInstance().setDefaultCommand(new ChassisMovement(joy));
    // joy.povDown()
    //     .and(() -> Constants.kIsTuningMode)
    //     .whileTrue(
    //         new QuestCalibration()
    //             .determineOffsetToRobotCenter(
    //                 SwerveSubsystem.getInstance(), RobotState.getInstance()::getQuestPose));
    // joy.povDown()
    //     .and(() -> !Constants.kIsTuningMode)
    //     .toggleOnTrue(
    //         new InstantCommand(
    //                 () -> {
    //                   Quest.setDisableQuest(!Quest.isDisableQuest());
    //                 })
    //             .ignoringDisable(true));

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

    // joy.leftBumper().whileTrue(new LockedChassisMovement(joy));
    // joy.a()
    //     .onTrue(
    //         SwerveSubsystem.getInstance()
    //             .followPosePathFinding(RobotState.getInstance().getClosestReefPose()));
    // joy.a().whileTrue(new MoveChassisToPose(true));

    joy.a()
        .onTrue(
            new InstantCommand(
                () -> {
                  SwerveSubsystem.getInstance()
                      .resetGyro(
                          SwerveSubsystem.getInstance().getGyroRotation().plus(Rotation2d.k180deg));
                  RobotState.getInstance()
                      .resetHeading(
                          SwerveSubsystem.getInstance().getGyroRotation().plus(Rotation2d.k180deg));
                }));
    // joy.rightBumper().whileTrue(new AlignChassisToPoseY(joy));

    joy.povDown().whileTrue(new AlignChassisToCenter(joy));
    joy.leftBumper().whileTrue(new AlignChassisToSide(joy, false));
    joy.rightBumper().whileTrue(new AlignChassisToSide(joy, true));

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

    joy.povLeft()
        .and(() -> Constants.kIsTuningMode)
        .whileTrue(wheelRadiusCharacterization(SwerveSubsystem.getInstance()));

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

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(SwerveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(0.05);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(0.25);
                  drive.setInputSpeeds(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getGyroRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * SwerveConstants.kDrivetrainRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
