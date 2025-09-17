package team5427.frc.robot.commands.chassis.assistance;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.Constants.OperatorConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.Vision.VisionSubsystem;

public class AlignChassisToCenter extends Command {
    private SwerveSubsystem swerve;
    private HolonomicDriveController driveController;

    private CommandXboxController joy;
    private TunedJoystick translationJoystick;

    private static Pose2d targetPose;
    
    public AlignChassisToCenter(CommandXboxController driverJoystick) {
        swerve = SwerveSubsystem.getInstance();
        driveController = new HolonomicDriveController(
            SwerveConstants.kTranslationXPIDController, 
            SwerveConstants.kTranslationYPIDController, 
            SwerveConstants.kRotationPIDController
        );

        joy = driverJoystick;
        translationJoystick = new TunedJoystick(joy.getHID());
        translationJoystick.useResponseCurve(ResponseCurve.LINEAR);

        translationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
        addRequirements(swerve);
    }
    
    public AlignChassisToCenter(CommandXboxController driverJoystick, Pose2d targetPose) {
        swerve = SwerveSubsystem.getInstance();
        joy = driverJoystick;
        translationJoystick = new TunedJoystick(joy.getHID());
        translationJoystick.useResponseCurve(ResponseCurve.LINEAR);
    
        translationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
        addRequirements(swerve);
    
        AlignChassisToCenter.targetPose = targetPose;
    
    }

    @Override
    public void initialize() {

        driveController =
            new HolonomicDriveController(
                SwerveConstants.kTranslationXPIDController,
                SwerveConstants.kTranslationYPIDController,
                SwerveConstants.kRotationPIDController);
        driveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(2)));

    }

    @Override
    public void execute() {
 
        if (DriverStation.isTeleop()) {
            targetPose = RobotState.getInstance().getAdaptivePose().nearest(
                List.of(RobotConfigConstants.kReefCenters)
            );
        }
        
        
        Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
        Logger.recordOutput("Target Pose", targetPose.relativeTo(robotPose));
        ChassisSpeeds speeds =
            // driveController.calculate(
            //     Pose2d.kZero,
            //     targetPose.relativeTo(robotPose),
            //     SwerveConstants.kAutoAlignTranslationalMaxSpeed,
            //     targetPose.getRotation().minus(robotPose.getRotation())
            // );
            driveController.calculate(
                Pose2d.kZero,
                targetPose.relativeTo(robotPose),
                SwerveConstants.kAutoAlignTranslationalMaxSpeed,
                targetPose.getRotation().minus(robotPose.getRotation())
            );
        if (driveController.getThetaController().atSetpoint() && 
                driveController.getXController().atSetpoint())
                    speeds = new ChassisSpeeds(0, 0, 0);

        double vx = translationJoystick.getRightY();
        double dampener = (joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount);

        ChassisSpeeds driverSpeeds =
        swerve.getDriveSpeeds(
            0.0,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            dampener);

        // establishes an aligned field relative speed for the x
        driverSpeeds.vxMetersPerSecond =
            swerve.getDriveSpeeds(
                    vx, 0.0, 0.0, dampener, targetPose.getRotation().plus(Rotation2d.k180deg))
                .vxMetersPerSecond;

        if (joy.getLeftTriggerAxis() >= 0.1) {
            driverSpeeds = new ChassisSpeeds(0, 0, 0);
        }
        
        swerve.setInputSpeeds(driverSpeeds);
        

    }

    @Override
    public boolean isFinished() {

      return false;
    }

    @Override
    public void end(boolean interrupted) {
      swerve.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}

