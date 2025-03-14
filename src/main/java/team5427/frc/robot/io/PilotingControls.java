package team5427.frc.robot.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;

import team5427.frc.robot.RobotState;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.commands.chassis.ChassisMovement;
import team5427.frc.robot.commands.chassis.LockedChassisMovement;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.Vision.VisionSubsystem;

public class PilotingControls {

  private CommandXboxController joy;

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

    SwerveSubsystem.getInstance(Optional.of(RobotState.getInstance()::addOdometryMeasurement)).setDefaultCommand(new ChassisMovement(joy));

    joy.leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  SwerveSubsystem.getInstance(null).setGyroLock(true);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  SwerveSubsystem.getInstance(null).setGyroLock(false);
                }));

    joy.leftStick().whileTrue(new LockedChassisMovement(joy, RobotConfigConstants.kReefPoses));

    joy.a()
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(
                    () -> {
                      RobotState
                          .getInstance().resetAllPose(VisionSubsystem.getInstance().getLatestPoseMeasurement().toPose2d());
                    }),
                new InstantCommand(),
                () -> {
                  return VisionSubsystem.getInstance().getLatestPoseMeasurement() != null;
                }));
    joy.y()
        .and(() -> RobotBase.isReal())
        .onTrue(
            new InstantCommand(
                () -> {
                  SwerveSubsystem.getInstance(null).resetGyro(Rotation2d.kZero);
                  RobotState.getInstance().resetHeading(Rotation2d.kZero);
                }));

    VisionSubsystem.getInstance(Optional.of(RobotState.getInstance()::addVisionMeasurement), Optional.of(RobotState.getInstance()::getEstimatedPose), Optional.of(RobotState.getInstance()::getOdometryHeading));
    // SwerveSubsystem.getInstance().setPose(VisionSubsystem.getInstance().getLatestPose().toPose2d());
  }
}
