package team5427.frc.robot.subsystems.Swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.Mode;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.SuperStructureEnum.DrivingStates;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIO;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIOPigeon;
import team5427.lib.kinematics.SwerveUtil;

public class SwerveSubsystem extends SubsystemBase {

    private static SwerveSubsystem m_instance;

    public static final Lock odometryLock = new ReentrantLock();
    private SwerveModule[] modules;
    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private ChassisSpeeds currentSpeeds;
    private SwerveModuleState[] actualModuleStates;

    public static DrivingStates state;

    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.m_kinematics,
            rawGyroRotation, lastModulePositions, new Pose2d());

    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);
    private boolean isFieldOp;

    public static SwerveSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new SwerveSubsystem();
        }
        return m_instance;
    }

    private SwerveSubsystem() {
        modules = new SwerveModule[4];

        modules[SwerveUtil.kFrontLeftModuleIdx] = new SwerveModule(SwerveUtil.kFrontLeftModuleIdx);
        modules[SwerveUtil.kFrontRightModuleIdx] = new SwerveModule(SwerveUtil.kFrontRightModuleIdx);
        modules[SwerveUtil.kRearLeftModuleIdx] = new SwerveModule(SwerveUtil.kRearLeftModuleIdx);
        modules[SwerveUtil.kRearRightModuleIdx] = new SwerveModule(SwerveUtil.kRearRightModuleIdx);

        switch (Constants.currentMode) {
            case REAL:
                gyroIO = new GyroIOPigeon();
                break;
            case REPLAY:
                break;
            case SIM:
                // gyroIO = new GyroIOSim();
                break;
            default:
                break;

        }
        isFieldOp = true;
        currentSpeeds = new ChassisSpeeds(0, 0, 0);

        PhoenixOdometryThread.getInstance().start();
        System.out.println("Created New Swerve");
    }

    public void setChassisSpeeds(ChassisSpeeds newSpeeds) {
        if (Constants.currentMode != Mode.SIM) {
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(newSpeeds, getGyroRotation());
        } else {
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(newSpeeds, getRotation());
        }

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < states.length; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    @Override
    public void periodic() {

        SwerveModuleState[] moduleStates = SwerveConstants.m_kinematics.toSwerveModuleStates(currentSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kDriveMotorConfiguration.maxVelocity);
        actualModuleStates = new SwerveModuleState[modules.length];
        odometryLock.lock(); // Prevents odometry updates while reading data
        if (gyroIO != null && gyroInputs.connected) {
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("Swerve/Gyro", gyroInputs);
        } else {
            gyroDisconnectedAlert.set(true);
        }
        for (int i = 0; i < modules.length; i++) {
            actualModuleStates[i] = modules[i].getModuleState();
            modules[i].setModuleState(moduleStates[i]);
            modules[i].periodic();
            // if (modules[i].getModuleState() != null) {

        }
        odometryLock.unlock();

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = SwerveConstants.m_kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
            super.periodic();
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
        Logger.recordOutput("SwerveOutput/ChassisSpeeds", currentSpeeds);
        Logger.recordOutput("SwerveOutput/ModulePositions", getModulePositions());
        Logger.recordOutput("SwerveOutput/ModuleStates", actualModuleStates);
        Logger.recordOutput("SwerveOutput/TargetModuleStates", moduleStates);
        Logger.recordOutput("Odometry/Robot", getPose());
    }

    public void resetGyro(Rotation2d angle) {
        gyroIO.resetGyroYawAngle(angle);
    }

    /**
     * Recommended to use
     * 
     * <pre>
     * getRotation()
     * </pre>
     * 
     * instead
     * 
     * @return Angle from the gyro
     */
    public Rotation2d getGyroRotation() {

        return gyroInputs.yawPosition;
    }

    public boolean getFieldOp() {
        return isFieldOp;
    }

    public void setFieldOp(boolean op) {
        isFieldOp = op;
    }

    public SwerveModuleState getModuleState(int index) {
        return modules[index].getModuleState();
    }

    public SwerveModulePosition getModulePosition(int index) {
        return modules[index].getModulePosition();
    }

    /**
     * @return Raw speed magnitude of the chassis
     */
    public double getSpeedMagnitude() {
        return Math.sqrt(Math.pow(SwerveConstants.m_kinematics.toChassisSpeeds(actualModuleStates).vxMetersPerSecond, 2)
                + Math.pow(SwerveConstants.m_kinematics.toChassisSpeeds(actualModuleStates).vyMetersPerSecond, 2));
    }

    // @AutoLogOutput(key = "SwerveOutput/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                getModulePosition(0), getModulePosition(1),
                getModulePosition(2),
                getModulePosition(3), };
    }

    /** Returns the current odometry pose. */
    // @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a new timestamped vision measurement. Meant to be a bridge method,
     * actual vision processing should be done in a seperate subsystem
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public void stop() {

        for (SwerveModule module : modules) {
            module.stop();
        }
    }
}
