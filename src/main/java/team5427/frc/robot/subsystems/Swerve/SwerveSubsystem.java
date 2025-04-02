package team5427.frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.Mode;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIO;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIOPigeon;
import team5427.lib.kinematics.SwerveUtil;

public class SwerveSubsystem extends SubsystemBase {

  public static final Lock odometryLock = new ReentrantLock();

  private SwerveModule[] swerveModules;
  @Getter private SwerveModuleState[] targetModuleStates;
  @Getter private SwerveModuleState[] actualModuleStates;
  @Getter private SwerveModulePosition[] modulePositions;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputsAutoLogged;

  @Getter private ChassisSpeeds inputSpeeds;

  // private SwerveSetpointGenerator setpointGenerator;

  // private SwerveSetpoint setpoint;

  private DriveFeedforwards driveFeedforwards;

  private OdometryConsumer odometryConsumer;

  private final Alert gyroDisconnectAlert =
      new Alert("Disconnected Gyro :(. Now using Kinematics", AlertType.kError);

  private static SwerveSubsystem m_instance;

  public static SwerveSubsystem getInstance() {
    return getInstance(null);
  }

  public static SwerveSubsystem getInstance(OdometryConsumer consumer) {
    if (m_instance == null) {
      m_instance = new SwerveSubsystem(consumer);
    }
    return m_instance;
  }

  private SwerveSubsystem(OdometryConsumer consumer) {
    swerveModules = new SwerveModule[SwerveUtil.kDefaultNumModules];

    swerveModules[SwerveUtil.kFrontLeftModuleIdx] =
        new SwerveModule(SwerveUtil.kFrontLeftModuleIdx);
    swerveModules[SwerveUtil.kFrontRightModuleIdx] =
        new SwerveModule(SwerveUtil.kFrontRightModuleIdx);
    swerveModules[SwerveUtil.kRearLeftModuleIdx] = new SwerveModule(SwerveUtil.kRearLeftModuleIdx);
    swerveModules[SwerveUtil.kRearRightModuleIdx] =
        new SwerveModule(SwerveUtil.kRearRightModuleIdx);

    targetModuleStates = new SwerveModuleState[SwerveUtil.kDefaultNumModules];
    for (int i = 0; i < SwerveUtil.kDefaultNumModules; i++)
      targetModuleStates[i] = new SwerveModuleState();
    actualModuleStates = new SwerveModuleState[targetModuleStates.length];

    modulePositions = new SwerveModulePosition[SwerveUtil.kDefaultNumModules];
    for (int i = 0; i < SwerveUtil.kDefaultNumModules; i++)
      modulePositions[i] = new SwerveModulePosition();

    inputSpeeds = new ChassisSpeeds();

    switch (Constants.currentMode) {
      case SIM:
      case REPLAY:
        // gyroIO = new GyroIOSim();
        break;
      case REAL:
        gyroIO = new GyroIOPigeon();
        break;
      default:
        // gyroIO = new GyroIOSim();
        break;
    }
    gyroInputsAutoLogged = new GyroIOInputsAutoLogged();

    odometryConsumer = null;
    if (consumer == null && odometryConsumer == null) {
      DriverStation.reportWarning("Swerve Subsystem not provided by OdometryConsumer", true);
    } else {
      odometryConsumer = consumer;
    }

    // setpointGenerator = new SwerveSetpointGenerator(Constants.config,
    // RotationsPerSecond.of(3.0));
    // setpoint = new SwerveSetpoint(inputSpeeds, actualModuleStates, driveFeedforwards);

    driveFeedforwards = null;

    PhoenixOdometryThread.getInstance().start();

    System.out.println("Created New Swerve");
  }

  public ChassisSpeeds getDriveSpeeds(
      double xInput, double yInput, double omegaInput, double dampenAmount) {

    xInput *= (1 - dampenAmount);
    yInput *= (1 - dampenAmount);
    omegaInput *= (1 - dampenAmount);

    ChassisSpeeds rawSpeeds =
        new ChassisSpeeds(
            xInput * SwerveConstants.kDriveMotorConfiguration.maxVelocity,
            yInput * SwerveConstants.kDriveMotorConfiguration.maxVelocity,
            omegaInput * SwerveConstants.kDriveMotorConfiguration.maxVelocity * Math.PI);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(rawSpeeds, getGyroRotation());
    // ChassisSpeeds fieldRelativeSpeeds = 
    //     ChassisSpeeds.fromRobotRelativeSpeeds(rawSpeeds, RobotState.getInstance().getAdaptivePose().getRotation());

    ChassisSpeeds discretizedSpeeds =
        ChassisSpeeds.discretize(fieldRelativeSpeeds, Constants.kLoopSpeed);

    return discretizedSpeeds;
  }

  public ChassisSpeeds getDriveSpeeds(
      double xInput, double yInput, Rotation2d targetOmega, double dampenAmount) {

    xInput *= (1 - dampenAmount);
    yInput *= (1 - dampenAmount);

    double calculatedOmega =
        SwerveConstants.kRotationPIDController.calculate(
            RobotState.getInstance().getAdaptivePose().getRotation().getRadians(),
            targetOmega.getRadians());

    ChassisSpeeds rawSpeeds =
        new ChassisSpeeds(
            xInput * SwerveConstants.kDriveMotorConfiguration.maxVelocity,
            yInput * SwerveConstants.kDriveMotorConfiguration.maxVelocity,
            calculatedOmega);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(rawSpeeds, getGyroRotation());
    // ChassisSpeeds fieldRelativeSpeeds = 
    //       ChassisSpeeds.fromRobotRelativeSpeeds(rawSpeeds, RobotState.getInstance().getAdaptivePose().getRotation());

    return fieldRelativeSpeeds;
  }

  public Command followPathCommand(Pose2d targetPose) { // UNTESTED

    Pose2d robotPose = RobotState.getInstance().getAdaptivePose();
    List<Waypoint> desiredWaypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(robotPose.getX(), robotPose.getY(), Rotation2d.kZero),
            new Pose2d(targetPose.getX(), targetPose.getY(), Rotation2d.kZero));

    PathPlannerPath generatedPath =
        new PathPlannerPath(
            desiredWaypoints,
            new PathConstraints(
                SwerveConstants.kDriveMotorConfiguration.maxVelocity,
                SwerveConstants.kDriveMotorConfiguration.maxAcceleration,
                2.0 * Math.PI,
                2.0 * Math.PI),
            null,
            new GoalEndState(0.0, targetPose.getRotation()));

    return new FollowPathCommand(
        generatedPath,
        RobotState.getInstance()::getAdaptivePose,
        this::getChassisSpeeds,
        this::setInputSpeeds,
        new PPHolonomicDriveController(
            new PIDConstants(SwerveConstants.kAutoAlignTranslationKp, 0.0, 0.0),
            new PIDConstants(SwerveConstants.kRotationalKp, 0.0, 0.0)),
        Constants.config,
        () -> false,
        this);
  }

  public Command followPosePathFinding(Pose2d pose) {
    if (Constants.kAlliance.get() == Alliance.Red) {
      return AutoBuilder.pathfindToPoseFlipped(pose, new PathConstraints(0.1, 1, 1, 2), 0);
    } else {
      return AutoBuilder.pathfindToPose(pose, new PathConstraints(0.1, 1, 1, 2), 0);
    }
  }

  public Rotation2d getGyroRotation() {
    return gyroInputsAutoLogged.yawPosition.unaryMinus();
  }

  public void resetGyro(Rotation2d newYaw) {
    gyroIO.resetGyroYawAngle(newYaw.unaryMinus());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.m_kinematics.toChassisSpeeds(actualModuleStates);
  }

  public SwerveModuleState[] getModuleStates() {
    return actualModuleStates;
  }

  @Override
  public void periodic() {

    // Lock Odometry (To prevent loss of data and inaccurate data updates)
    odometryLock.lock();
    if (gyroIO != null) {
      gyroIO.updateInputs(gyroInputsAutoLogged);
      Logger.processInputs("Swerve/Gyro", gyroInputsAutoLogged);
    } else {
      gyroDisconnectAlert.set(true);
    }

    // Create New Target Module States from inputSpeeds
    targetModuleStates = SwerveConstants.m_kinematics.toSwerveModuleStates(inputSpeeds);

    for (int i = 0; i < swerveModules.length; i++) {
      if (driveFeedforwards != null) {
        swerveModules[i].setModuleState(
            targetModuleStates[i], driveFeedforwards); // Set new target module state
      } else {
        swerveModules[i].setModuleState(targetModuleStates[i]);
      }
      actualModuleStates[i] = swerveModules[i].getModuleState(); // Read actual module state
      swerveModules[i].periodic(); // Update Module Inputs
    }
    // Unlock Odometry
    odometryLock.unlock();

    // Update Odometry
    double[] odometrySampleTimestamps = swerveModules[0].getOdometryTimestamps();
    int initialTimestampLength = odometrySampleTimestamps.length;
    for (int i = 0; i < initialTimestampLength; i++) {
      SwerveModulePosition[] newModulePositions = new SwerveModulePosition[modulePositions.length];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modulePositions.length];
      for (int midx = 0; midx < modulePositions.length; midx++) {
        newModulePositions[midx] = swerveModules[midx].getOdometryPositions()[i];
        moduleDeltas[midx] =
            new SwerveModulePosition(
                newModulePositions[midx].distanceMeters - modulePositions[midx].distanceMeters,
                newModulePositions[midx].angle);
        modulePositions[midx] = newModulePositions[midx];
      }

      Rotation2d newGyroInput = Rotation2d.kZero;
      if (gyroInputsAutoLogged.connected && gyroIO != null) {
        newGyroInput = gyroInputsAutoLogged.odometryYawPositions[i].plus(Rotation2d.k180deg);
      } else {
        Twist2d gyroTwist = SwerveConstants.m_kinematics.toTwist2d(moduleDeltas);
        newGyroInput = newGyroInput.plus(Rotation2d.fromRadians(gyroTwist.dtheta));
      }

      odometryConsumer.accept(odometrySampleTimestamps[i], newGyroInput, newModulePositions);
    }

    gyroDisconnectAlert.set(!gyroInputsAutoLogged.connected && Constants.currentMode != Mode.SIM);

    Logger.recordOutput("SwerveOutput/InputSpeeds", inputSpeeds);
    Logger.recordOutput("SwerveOutput/ModulePositions", getModulePositions());
    Logger.recordOutput("SwerveOutput/ModuleStates", actualModuleStates);
    Logger.recordOutput("SwerveOutput/TargetModuleStates", targetModuleStates);
  }

  public void setInputSpeeds(ChassisSpeeds speeds, DriveFeedforwards driveFeedforwards) {
    this.inputSpeeds = speeds;
    this.driveFeedforwards = driveFeedforwards;
  }

  public void setInputSpeeds(ChassisSpeeds speeds) {
    this.inputSpeeds = speeds;
    this.driveFeedforwards = null;
  }

  @FunctionalInterface
  public static interface OdometryConsumer {
    public void accept(
        double timestampSeconds, Rotation2d rotation, SwerveModulePosition[] modulePositions);
  }
}
