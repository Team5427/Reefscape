package team5427.frc.robot.commands.chassis.assistance;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AlignChassisFromPolar extends Command{
    private SwerveSubsystem swerve;
    private HolonomicDriveController driveController;
    private CommandXboxController joy;
    private TunedJoystick translationalJoystick;
    private static Pose2d targetPose;
    private boolean isRight;
    public AlignChassisFromPolar(CommandXboxController driverJoystick, boolean isRight){
        swerve = SwerveSubsystem.getInstance();
        driveController = new HolonomicDriveController(SwerveConstants.kTranslationXPIDController, SwerveConstants.kTranslationYPIDController, SwerveConstants.kRotationPIDController);
        driveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromRadians(Math.PI/72)));
        
        joy = driverJoystick;
        translationalJoystick = new TunedJoystick(joy.getHID());

    }
}