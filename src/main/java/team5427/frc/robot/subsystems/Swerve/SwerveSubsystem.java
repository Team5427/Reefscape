package team5427.frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.Mode;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.SuperStructureEnum.DrivingStates;
import team5427.frc.robot.commands.chassis.ChassisMovement;
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
  private ChassisSpeeds inputSpeeds;
  private SwerveModuleState[] actualModuleStates;
  private boolean bypass = false;
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private SwerveModuleState[] moduleStates = new SwerveModuleState[4];

  private boolean isStopped = false;

  private final SysIdRoutine sysId;

  public static DrivingStates state;

  private boolean gyroLock = false;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveDrivePoseEstimator poseEstimator;

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private boolean isFieldOp;

  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

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
        gyroIO = new GyroIOPigeon();
        break;
    }
    isFieldOp = true;
    inputSpeeds = new ChassisSpeeds(0, 0, 0);
    PhoenixOdometryThread.getInstance().start();

    actualModuleStates = getModuleStates();
    // modules[0].setModuleState(new SwerveModuleState(0,Rotation2d.kZero));
    if (Constants.config == null) System.out.println("Robot Config is null");
    setpointGenerator =
        new SwerveSetpointGenerator(
            Constants.config,
            RotationsPerSecond.of(SwerveConstants.kSteerMotorConfiguration.maxVelocity / 60.0));
    previousSetpoint =
        new SwerveSetpoint(inputSpeeds, actualModuleStates, DriveFeedforwards.zeros(4));

    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.m_kinematics,
            rawGyroRotation,
            lastModulePositions,
            Pose2d.kZero,
            VecBuilder.fill(0.6, 0.6, 0.1),
            VecBuilder.fill(0.2, 0.2, 0.7));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(7.0),
                Volts.of(70),
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDriveCharacterization(voltage.in(Volts)), null, this));
    System.out.println("Created New Swerve");
  }

  public void setChassisSpeeds(ChassisSpeeds newSpeeds) {
    this.inputSpeeds = newSpeeds;
  }

  public void setSpeedsAuton(ChassisSpeeds speeds) {
    setChassisSpeeds(speeds);
  }

  // public void setChassisSpeeds(ChassisSpeeds newSpeeds, DriveFeedforwards feedforwards) {
  //   this.inputSpeeds = newSpeeds;
  // }

  public void setFieldRelativeSpeeds(ChassisSpeeds newSpeeds) {}

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < states.length; i++) {
      states[i] = modules[i].getModuleState();
    }
    return states;
  }

  @Override
  public void periodic() {
    // PhoenixOdometryThread.getInstance().run();

    if (bypass) return;

    odometryLock.lock(); // Prevents odometry updates while reading data
    if (gyroIO != null) {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Swerve/Gyro", gyroInputs);
    } else {
      gyroDisconnectedAlert.set(true);
    }

    if (DriverStation.isAutonomous()) isStopped = false;
    ChassisSpeeds relativeSpeeds;
    if (gyroLock) {
      inputSpeeds.omegaRadiansPerSecond = 0;
    }
    // this, and the relavant methods it calls are the sources of error
    // relativeSpeeds =
    //     Constants.currentMode != Mode.SIM
    //         ? ChassisSpeeds.fromRobotRelativeSpeeds(inputSpeeds, getGyroRotation())
    //         : ChassisSpeeds.fromRobotRelativeSpeeds(inputSpeeds, getRotation());
    relativeSpeeds = DriverStation.isAutonomous() 
        ? inputSpeeds
        : ChassisSpeeds.fromRobotRelativeSpeeds(inputSpeeds, getGyroRotation());
    // previousSetpoint =
    //     setpointGenerator.generateSetpoint(
    //         previousSetpoint, // The previous setpoint
    //         relativeSpeeds, // The desired target speeds
    //         Seconds.of(Constants.kLoopSpeed), Volts.of( RobotController.getBatteryVoltage()) //
    // The loop time of the robot code, in seconds
    //         );
    // if (DriverStation.isTeleop() && false) {
    //   previousSetpoint =
    //       setpointGenerator.generateSetpoint(
    //           previousSetpoint, // The previous setpoint
    //           relativeSpeeds, // The desired target speeds
    //           Seconds.of(Constants.kLoopSpeed),
    //           Volts.of(
    //               RobotController.getBatteryVoltage()) // The loop time of the robot code, in
    // seconds
    //           );
    //   moduleStates = previousSetpoint.moduleStates();
    // } else {
    ChassisSpeeds discretizedSpeeds =
        ChassisSpeeds.discretize(relativeSpeeds, Constants.kLoopSpeed);

    moduleStates = SwerveConstants.m_kinematics.toSwerveModuleStates(discretizedSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     moduleStates, SwerveConstants.kDriveMotorConfiguration.maxVelocity);
    // }
    actualModuleStates = new SwerveModuleState[modules.length];
    if (isStopped) {
      setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    for (int i = 0; i < modules.length; i++) {
      if (isStopped) {
        modules[i].stop();
      } else {
        modules[i].setModuleState(moduleStates[i]);
      }
      actualModuleStates[i] = modules[i].getModuleState();
      modules[i].periodic();
    }
    odometryLock.unlock();

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i].unaryMinus();
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = SwerveConstants.m_kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    Logger.recordOutput("SwerveOutput/RobotRelativeChassisSpeeds", inputSpeeds);
    Logger.recordOutput("SwerveOutput/DiscretizedChassisSpeeds", discretizedSpeeds);
    Logger.recordOutput("SwerveOutput/ModulePositions", getModulePositions());
    Logger.recordOutput("SwerveOutput/ModuleStates", actualModuleStates);
    Logger.recordOutput("SwerveOutput/TargetModuleStates", moduleStates);
    Logger.recordOutput("Odometry/Robot", getPose());
  }

  public void setGyroLock(boolean setLock) {
    this.gyroLock = setLock;
  }

  public boolean getGyroLock() {
    return this.gyroLock;
  }

  public void resetGyro(Rotation2d angle) {
    gyroIO.resetGyroYawAngle(angle);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runDriveCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runDriveCharacterization(output);
    }
  }

  /** Runs the steer with the specified steer output. */
  public void runSteerCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runSteerCharacterization(output);
    }
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

    return gyroInputs.yawPosition.unaryMinus();
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
    return Math.sqrt(
        Math.pow(
                SwerveConstants.m_kinematics.toChassisSpeeds(actualModuleStates).vxMetersPerSecond,
                2)
            + Math.pow(
                SwerveConstants.m_kinematics.toChassisSpeeds(actualModuleStates).vyMetersPerSecond,
                2));
  }

  // @AutoLogOutput(key = "SwerveOutput/ModulePositions")
  public SwerveModulePosition[] getModulePositions() {

    this.modulePositions[0] = getModulePosition(0);
    this.modulePositions[1] = getModulePosition(1);
    this.modulePositions[2] = getModulePosition(2);
    this.modulePositions[3] = getModulePosition(3);
    return this.modulePositions;
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    return SwerveConstants.m_kinematics.toChassisSpeeds(getModuleStates());
    // return inputSpeeds;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(getCurrentChassisSpeeds(), getGyroRotation());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    // poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    poseEstimator.resetPose(pose);
    resetGyro(poseEstimator.getEstimatedPosition().getRotation());
  }

  public void resetAutonPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /**
   * Adds a new timestamped vision measurement. Meant to be a bridge method, actual vision
   * processing should be done in a seperate subsystem
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /**
   * Returns a command to run a quasistatic test in the specified direction. Change out the method
   * name to run a different sys id test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // return run(() -> runDriveCharacterization(80.0))
    //     .withTimeout(1.0)
    //     .andThen(sysId.quasistatic(direction));
    return sysId.quasistatic(direction);
  }

  /**
   * Returns a command to run a dynamic test in the specified direction. Change out the method name
   * to run a different sys id test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // return run(() -> runDriveCharacterization(80.0))
    //     .withTimeout(1.0)
    //     .andThen(sysId.dynamic(direction));
    return sysId.dynamic(direction);
  }

  /** Returns the position of each module in radians. */
  public Angle[] getWheelRadiusCharacterizationPositions() {
    Angle[] values = new Angle[4];
    for (int i = 0; i < 4; i++) {
      values[i] = Rotations.of(modules[i].getWheelRadiusCharacterizationPosition().getRotations());
    }
    return values;
  }

  public void stop(boolean enabled) {
    this.isStopped = enabled;
  }

  public void bypass(boolean bypass) {
    this.bypass = bypass;
  }
}
