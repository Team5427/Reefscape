package team5427.frc.robot.io;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.SuperStructureEnum.EndEffectorStates;
import team5427.frc.robot.commands.ChassisMovement;
import team5427.frc.robot.commands.CoralIntakeTest;
import team5427.frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class PilotingControls {

    private CommandXboxController joy;

    public PilotingControls() {
        joy = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        SwerveSubsystem.getInstance().setDefaultCommand(new ChassisMovement(joy));

        joy.a().onTrue(new CoralIntakeTest()).onFalse(new InstantCommand(() -> {
            EndEffectorSubsystem.getInstance().setAlgaeRollerSetpoint(MetersPerSecond.of(0.0));
            EndEffectorSubsystem.getInstance().setCoralRollerSetpoint(MetersPerSecond.of(0.0));
            EndEffectorSubsystem.getInstance().setPivotSetpoint(Rotation2d.fromDegrees(10.0));
            EndEffectorSubsystem.getInstance().setWristSetpoint(Rotation2d.fromDegrees(10.0));
            EndEffectorSubsystem.state = EndEffectorStates.IDLE;
        }));

        joy.y().onTrue(new InstantCommand(() -> {
            SwerveSubsystem.getInstance().resetGyro(new Rotation2d());
            ;
        }));
    }

}