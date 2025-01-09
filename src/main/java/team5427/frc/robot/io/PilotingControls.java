package team5427.frc.robot.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.commands.ChassisMovement;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class PilotingControls {


    private CommandXboxController joy;

    public PilotingControls() {
        joy = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        SwerveSubsystem.getInstance().setDefaultCommand(new ChassisMovement(joy));
    
        joy.y().onTrue(new InstantCommand(() -> {
            SwerveSubsystem.getInstance().resetGyro(new Rotation2d());
            ;
        }));
    }

}