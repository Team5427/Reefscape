package team5427.frc.robot.io;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.SuperStructureEnum.EndEffectorStates;
import team5427.frc.robot.commands.CascadeTest;
import team5427.frc.robot.commands.ChassisMovement;
import team5427.frc.robot.commands.CoralIntakeTest;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class PilotingControls {

  private CommandXboxController joy;

  public PilotingControls() {
    joy = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    // SwerveSubsystem.getInstance().setDefaultCommand(new ChassisMovement(joy));

    joy.a()
        .onTrue(new CoralIntakeTest())
        .whileFalse(
            new InstantCommand(
                () -> {
                  EndEffectorSubsystem.state = EndEffectorStates.IDLE;
                  EndEffectorSubsystem.getInstance()
                      .setAlgaeRollerSetpoint(MetersPerSecond.of(0.0));
                  EndEffectorSubsystem.getInstance()
                      .setCoralRollerSetpoint(MetersPerSecond.of(0.0));
                  EndEffectorSubsystem.getInstance()
                      .setPivotSetpoint(Rotation2d.fromDegrees(-10.0));
                  EndEffectorSubsystem.getInstance().setWristSetpoint(Rotation2d.fromDegrees(0.0));
                },
                EndEffectorSubsystem.getInstance()));

    joy.b().whileTrue(new CascadeTest()).whileFalse(new InstantCommand(() -> {
      // CascadeSubsystem.getInstance().setCascadeEncoderPosition(Distance.ofBaseUnits(0.0,
      // Meters));
      CascadeSubsystem.getInstance().setCascadeSetpoint(Distance.ofRelativeUnits(.5, Meters));
      CascadeSubsystem.getInstance().setPivotSetpoint(Rotation2d.fromDegrees(0.1));

    }));

    joy.leftTrigger().negate()
        .whileFalse(
            new InstantCommand(
                () -> {
                  SwerveSubsystem.getInstance().getCurrentCommand().end(true);
                  SwerveSubsystem.getInstance().setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
                },
                SwerveSubsystem.getInstance()))
        .whileTrue(new ChassisMovement(joy));

    if (DriverStation.isDisabled()) {
    }

    joy.y().and(() -> RobotBase.isReal())
        .onTrue(
            new InstantCommand(
                () -> {
                  SwerveSubsystem.getInstance().resetGyro(new Rotation2d());
                  ;
                }));
  }
}
