package team5427.frc.robot.commands.chassis.assistance;

import java.util.ArrayList;
import java.util.List;

import org.checkerframework.checker.units.qual.t;
import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private ProfiledPIDController distanceController;
    private ProfiledPIDController angleController;
    private TunedJoystick translationJoystick;
    private CommandXboxController joy;

    private static Pose2d targetPose;
    public AlignChassisFromPolar(CommandXboxController driverJoystick, boolean isRight, Pose2d targetPose){
        swerve = SwerveSubsystem.getInstance();
        
        distanceController = SwerveConstants.kAutoAlignServoController;
        distanceController.setTolerance(0.02);

        angleController = SwerveConstants.kRotationPIDController;

        angleController.enableContinuousInput(-Math.PI, Math.PI); 
        angleController.setTolerance(Math.toRadians(1));

        translationJoystick = new TunedJoystick(driverJoystick.getHID());
        translationJoystick.useResponseCurve(ResponseCurve.LINEAR);
        translationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
        addRequirements(swerve);
        AlignChassisFromPolar.targetPose = targetPose;
    }
    @Override
    public void initialize(){
        distanceController.reset(0);
        angleController.reset(0);
    }

    @Override
    public void execute(){
        Logger.recordOutput("Target Pose", targetPose);
        Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
        Logger.recordOutput("Relative Target Pose", targetPose.relativeTo(robotPose));
        Translation2d relativeVector = targetPose.getTranslation().minus(robotPose.getTranslation());
        double distance = relativeVector.getNorm();
        Rotation2d targetAngle = relativeVector.getAngle();
        double radialVelocity = distanceController.calculate(distance, 0);
        double angularVelocity = angleController.calculate(robotPose.getRotation().getRadians(), targetAngle.getRadians());
        double vx = radialVelocity * targetAngle.getCos();
        double vy = radialVelocity * targetAngle.getSin();
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, angularVelocity);
        swerve.setInputSpeeds(speeds);
        double dampener = (joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount);

        ChassisSpeeds driverSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, robotPose.getRotation()
            );
        driverSpeeds.vxMetersPerSecond = swerve.getDriveSpeeds(-vx, 0.0, 0.0, dampener, targetPose.getRotation()).vxMetersPerSecond;
        swerve.setInputSpeeds(driverSpeeds);


    }
    @Override
    public boolean isFinished(){
        return distanceController.atSetpoint() && angleController.atSetpoint();

    }
    @Override
    public void end(boolean interrupted){
        swerve.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}