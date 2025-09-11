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

public class AlignChassisToSide extends Command {
    private SwerveSubsystem swerve;
    private HolonomicDriveController driveController;

    private CommandXboxController joy;
    private TunedJoystick translationJoystick;

    private static Pose2d targetPose;

    private boolean isRight;
    
    public AlignChassisToSide(CommandXboxController driverJoystick, boolean isRight) {
        swerve = SwerveSubsystem.getInstance();
        driveController = new HolonomicDriveController(
            SwerveConstants.kTranslationXPIDController, 
            SwerveConstants.kTranslationYPIDController, 
            SwerveConstants.kRotationPIDController
        );
        driveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1)));

        joy = driverJoystick;
        translationJoystick = new TunedJoystick(joy.getHID());
        translationJoystick.useResponseCurve(ResponseCurve.LINEAR);

        translationJoystick.setDeadzone(OperatorConstants.kDriverControllerJoystickDeadzone);
        addRequirements(swerve);

        this.isRight = isRight;

        if (DriverStation.isTeleop()) {
            List<Pose2d> actualPoses;
            List<Pose2d> targetPoses = new ArrayList<>();
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesBlue));
            } else {
                actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesRed));
            }
            for (int i = isRight ? 0 : 1; i<actualPoses.size(); i+=2) {
                targetPoses.add(actualPoses.get(i));
                System.out.println(actualPoses.get(i));
            }
            targetPose = RobotState.getInstance().getAdaptivePose().nearest(targetPoses);
        }
    }

    @Override
    public void initialize() {
        if (DriverStation.isTeleop()) {
            List<Pose2d> actualPoses;
            List<Pose2d> targetPoses = new ArrayList<>();
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesBlue));
            } else {
                actualPoses = List.copyOf(List.of(RobotConfigConstants.kAlignPosesRed));
            }
            for (int i = isRight ? 0 : 1; i<actualPoses.size(); i+=2) {
                targetPoses.add(actualPoses.get(i));
                System.out.println(actualPoses.get(i));
            }
            targetPose = RobotState.getInstance().getAdaptivePose().nearest(targetPoses);
        }
       
    }

    @Override
    public void execute() {
        Logger.recordOutput("Target Pose", targetPose);
        Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
        Logger.recordOutput("Relative Target Pose", targetPose.relativeTo(robotPose));
        ChassisSpeeds speeds =
            driveController.calculate(
                Pose2d.kZero,
                targetPose.relativeTo(robotPose),
                SwerveConstants.kAutoAlignTranslationalMaxSpeed,
                targetPose.relativeTo(robotPose).getRotation()
            );
        if(Math.abs(robotPose.getRotation().getDegrees())<90){
            speeds.vyMetersPerSecond *= -1;

        }
        if(DriverStation.getAlliance().get()==Alliance.Blue){
            speeds.vyMetersPerSecond*=-1;
        }
        if (driveController.getThetaController().atSetpoint() && 
            driveController.getYController().atSetpoint())
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
        // driverSpeeds.vxMetersPerSecond =
        //     swerve.getDriveSpeeds(
        //             vx, 0.0, 0.0, dampener, targetPose.getRotation())
        //         .vxMetersPerSecond;
        //         if (joy.getLeftTriggerAxis() >= 0.1) {
        //             driverSpeeds = new ChassisSpeeds(0, 0, 0);
        //           }

        driverSpeeds.vxMetersPerSecond = swerve.getDriveSpeeds(-vx, 0.0, 0.0, dampener, targetPose.getRotation()).vxMetersPerSecond;

        
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
