package team5427.frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ChassisMovement extends Command {

    private SwerveSubsystem swerveSubsystem;
    private CommandXboxController joy;
    private Optional<CommandXboxController> simulatedRotationalJoy;

    public ChassisMovement(CommandXboxController driverJoystick) {
        swerveSubsystem = SwerveSubsystem.getInstance();
        joy = driverJoystick;
        // if (RobotBase.isSimulation()) {
        //     simulatedRotationalJoy = Optional.of(new CommandXboxController(1));
        // }
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.setFieldOp(DriverStation.getAlliance().get().equals(Alliance.Red));
    }

    @Override
    public void execute() {
        if (false && RobotBase.isSimulation()) {
            double vx = -joy.getRawAxis(0) * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
            double vy = -joy.getRawAxis(1) * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
            double omegaRadians = -simulatedRotationalJoy.get().getRawAxis(0) * Math.PI * 4.0;
            ChassisSpeeds inputSpeeds = new ChassisSpeeds(vx, vy, omegaRadians);
            swerveSubsystem.setChassisSpeeds(inputSpeeds);
            Logger.recordOutput("InputSpeeds", inputSpeeds);
        } else {

            double vx = -joy.getRightY() * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
            double vy = -joy.getRightX() * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
            double omegaRadians = -joy.getLeftX() * Math.PI * SwerveConstants.kDriveMotorConfiguration.maxVelocity / 2;

            ChassisSpeeds inputSpeeds = new ChassisSpeeds(vx, vy, omegaRadians);


            swerveSubsystem.setChassisSpeeds(inputSpeeds);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}