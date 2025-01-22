package team5427.frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.drivers.ComplexGearRatio;
import team5427.lib.kinematics.SwerveUtil;
import team5427.lib.motors.MotorUtil;
import team5427.lib.motors.real.MotorConfiguration;
import team5427.lib.motors.real.MotorConfiguration.IdleState;
import team5427.lib.motors.real.MotorConfiguration.MotorMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kLoopSpeed = 0.020;
  public static final String kCanivoreCanBusName = "dummy";
  public static final double kOdometryFrequency =
      new CANBus(Constants.kCanivoreCanBusName).isNetworkFD() ? 250.0 : 100.0;
  // public static final double kOdometryFrequency = 100; // hz - so every 10 ms
  public static Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double driverControllerJoystickDeadzone = 0.00;
  }

  public static class SwerveConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.88);
    public static final double kTrackWidth = Units.inchesToMeters(19.5);
    public static final double kWheelBase = Units.inchesToMeters(19.5);

    public static final PIDController kSIMSteerController = new PIDController(1, 0, 0.0);
    public static final SimpleMotorFeedforward kSIMSteerFeedforward =
        new SimpleMotorFeedforward(0, 0, 0);

    public static final PIDController kSIMDriveController = new PIDController(2, 0, 0);
    public static final SimpleMotorFeedforward kSIMDriveFeedforward =
        new SimpleMotorFeedforward(0., 4, 0.2);

    public static final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveUtil kSwerveUtilInstance = new SwerveUtil();

    static {
      kSIMSteerController.enableContinuousInput(-Math.PI, Math.PI);
      kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(5, "*");
      kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(3, "*");
      kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(9, "*");
      kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(7, "*");

      kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(6);
      kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(4);
      kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(10);
      kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(8);

      kSwerveUtilInstance.kCancoderIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(12, "*");
      kSwerveUtilInstance.kCancoderIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(13, "*");
      kSwerveUtilInstance.kCancoderIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(14, "*");
      kSwerveUtilInstance.kCancoderIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(15, "*");

      kSwerveUtilInstance.kDriveInversion[SwerveUtil.kFrontLeftModuleIdx] = false;
      kSwerveUtilInstance.kDriveInversion[SwerveUtil.kFrontRightModuleIdx] = false;
      kSwerveUtilInstance.kDriveInversion[SwerveUtil.kRearLeftModuleIdx] = false;
      kSwerveUtilInstance.kDriveInversion[SwerveUtil.kRearRightModuleIdx] = false;

      kSwerveUtilInstance.kSteerInversion[SwerveUtil.kFrontLeftModuleIdx] = true;
      kSwerveUtilInstance.kSteerInversion[SwerveUtil.kFrontRightModuleIdx] = true;
      kSwerveUtilInstance.kSteerInversion[SwerveUtil.kRearLeftModuleIdx] = true;
      kSwerveUtilInstance.kSteerInversion[SwerveUtil.kRearRightModuleIdx] = true;

      kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kFrontLeftModuleIdx] = 0.48;

      kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kFrontRightModuleIdx] = 0.255;

      kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kRearLeftModuleIdx] = -0.38;

      kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kRearRightModuleIdx] = 0.278;
    }

    public static final MotorConfiguration kDriveMotorConfiguration = new MotorConfiguration();

    static {
      kDriveMotorConfiguration.gearRatio = SwerveUtil.kSDSL3GearRatio;
      kDriveMotorConfiguration.idleState = IdleState.kBrake;
      kDriveMotorConfiguration.mode = MotorMode.kFlywheel;

      kDriveMotorConfiguration.currentLimit = 80;
      kDriveMotorConfiguration.finalDiameterMeters = kWheelDiameterMeters;

      kDriveMotorConfiguration.maxVelocity =
          kDriveMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM);
      kDriveMotorConfiguration.maxAcceleration = kDriveMotorConfiguration.maxVelocity * 2.0;

      kDriveMotorConfiguration.kP = 0.75;
      kDriveMotorConfiguration.kV = 2.08;
      kDriveMotorConfiguration.kA = 0.20;
      kDriveMotorConfiguration.altV = kDriveMotorConfiguration.maxVelocity;
      kDriveMotorConfiguration.altA = kDriveMotorConfiguration.maxAcceleration;
    }

    public static final MotorConfiguration kSteerMotorConfiguration = new MotorConfiguration();

    static {
      kSteerMotorConfiguration.gearRatio = SwerveUtil.kSDSSteerGearRatioMK4n;
      kSteerMotorConfiguration.idleState = IdleState.kBrake;
      kSteerMotorConfiguration.mode = MotorMode.kServo;
      kSteerMotorConfiguration.currentLimit = 30;

      kSteerMotorConfiguration.maxVelocity =
          kSteerMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM);
      kSteerMotorConfiguration.maxAcceleration = kSteerMotorConfiguration.maxVelocity * 1000.0;

      // Tunable values
      kSteerMotorConfiguration.kP = 0.7; // 7.0
      kSteerMotorConfiguration.kD = 0.15;
      kSteerMotorConfiguration.altV = kSteerMotorConfiguration.maxVelocity;
      kSteerMotorConfiguration.altA = kSteerMotorConfiguration.maxAcceleration;
    }

    public static final CANDeviceId kPigeonCANId = new CANDeviceId(16, "*");

    public static class SimulationConstants {
      public static final double steerkP = 30.0;
      public static final double steerkI = 0.0;
      public static final double steerkD = 2.0;

      public static final double drivekP = 20.0;
      public static final double drivekI = 0.0;
      public static final double drivekD = 0.0;
      public static final double drivekS = 0.0;
      public static final double drivekV = 45.0;
    }
  }

  public static class EndEffectorConstants {
    public static final CANDeviceId kPivotMotorCanID = new CANDeviceId(0);
    public static final CANDeviceId kWristMotorCanID = new CANDeviceId(0);
    public static final CANDeviceId kCoralRollerMotorCanID = new CANDeviceId(0);
    public static final CANDeviceId kAlgaeRollerMotorCanID = new CANDeviceId(0);

    public static final CANDeviceId kWristCancoderCanID = new CANDeviceId(0);
    public static final CANDeviceId kPivotCancoderCanID = new CANDeviceId(0);

    public static final double kCoralWheelDiameter = Units.inchesToMeters(4.00);
    public static final double kAlgaeWheelDiameter = Units.inchesToMeters(4.00);

    public static ProfiledPIDController kSIMPivotController =
        new ProfiledPIDController(1.1, 0, 1, (new Constraints(10, 20)));
    public static ProfiledPIDController kSIMWristController =
        new ProfiledPIDController(1.1, 0, 1, new Constraints(10, 20));

    public static MotorConfiguration kPivotMotorConfiguration = new MotorConfiguration();
    public static MotorConfiguration kWristMotorConfiguration = new MotorConfiguration();
    public static MotorConfiguration kCoralRollerMotorConfiguration = new MotorConfiguration();
    public static MotorConfiguration kAlgaeRollerMotorConfiguration = new MotorConfiguration();

    public static final Rotation2d kWristMinimumAngle = new Rotation2d(0);
    public static final Rotation2d kWristMaximumAngle = new Rotation2d(180);
    public static final Rotation2d kPivotMinimumAngle = new Rotation2d(0);
    public static final Rotation2d kPivotMaximumAngle = new Rotation2d(180);

    public static final ComplexGearRatio kWristGearRatio = new ComplexGearRatio((1.0));
    public static final ComplexGearRatio kPivotGearRatio = new ComplexGearRatio((1.0));
    public static final ComplexGearRatio kCoralRollerGearRatio = new ComplexGearRatio((1.0));
    public static final ComplexGearRatio kAlgaeRollerGearRatio = new ComplexGearRatio((1.0));

    public static final double kWristCancoderOffset = 0.0;
    public static final double kPivotCancoderOffset = 0.0;

    static {
      kPivotMotorConfiguration.gearRatio = kPivotGearRatio;
      kPivotMotorConfiguration.currentLimit = 40;
      kPivotMotorConfiguration.idleState = IdleState.kBrake;
      kPivotMotorConfiguration.mode = MotorMode.kServo;
      kPivotMotorConfiguration.isInverted = false;

      kPivotMotorConfiguration.withFOC = true;

      kPivotMotorConfiguration.maxVelocity =
          kPivotMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM);
      kPivotMotorConfiguration.maxAcceleration = kPivotMotorConfiguration.maxVelocity * 2.0;

      kPivotMotorConfiguration.altA = kPivotMotorConfiguration.maxAcceleration;
      kPivotMotorConfiguration.altA = kPivotMotorConfiguration.maxVelocity;
      kPivotMotorConfiguration.kG = 0.27;
      kPivotMotorConfiguration.kA = 0.02;
      kPivotMotorConfiguration.kV = 4.34;

      kPivotMotorConfiguration.kP = 0.1;

      kPivotMotorConfiguration.tolerance = 0.1;
    }

    static {
      kWristMotorConfiguration.gearRatio = kPivotGearRatio;
      kWristMotorConfiguration.currentLimit = 40;
      kWristMotorConfiguration.idleState = IdleState.kBrake;
      kWristMotorConfiguration.mode = MotorMode.kServo;
      kWristMotorConfiguration.isInverted = false;

      kPivotMotorConfiguration.withFOC = true;

      kWristMotorConfiguration.maxVelocity =
          kWristMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM);
      kWristMotorConfiguration.maxAcceleration = kWristMotorConfiguration.maxVelocity * 2.0;

      kWristMotorConfiguration.altA = kWristMotorConfiguration.maxAcceleration;
      kWristMotorConfiguration.altA = kWristMotorConfiguration.maxVelocity;
      kWristMotorConfiguration.kA = 0.02;
      kWristMotorConfiguration.kV = 4.34;

      kWristMotorConfiguration.kP = 0.1;

      kWristMotorConfiguration.tolerance = 0.1;
    }

    static {
      kCoralRollerMotorConfiguration.currentLimit = 30;
      kCoralRollerMotorConfiguration.gearRatio = kCoralRollerGearRatio;
      kCoralRollerMotorConfiguration.isInverted = false;
      kCoralRollerMotorConfiguration.mode = MotorMode.kFlywheel;
      kCoralRollerMotorConfiguration.idleState = IdleState.kCoast;
      kCoralRollerMotorConfiguration.finalDiameterMeters = kCoralWheelDiameter;

      kPivotMotorConfiguration.withFOC = false;

      kCoralRollerMotorConfiguration.maxVelocity =
          kCoralRollerMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKraken_MaxRPM);
      kCoralRollerMotorConfiguration.maxAcceleration =
          kCoralRollerMotorConfiguration.maxVelocity * 2.0;

      kCoralRollerMotorConfiguration.altA = kCoralRollerMotorConfiguration.maxAcceleration;
      kCoralRollerMotorConfiguration.altV = kCoralRollerMotorConfiguration.maxVelocity;

      kCoralRollerMotorConfiguration.kP = 0.1;
      kCoralRollerMotorConfiguration.kA = 0.1;
      kCoralRollerMotorConfiguration.kV = 0.1;
      kCoralRollerMotorConfiguration.kD = 0.1;

      kCoralRollerMotorConfiguration.tolerance = 0.05;
    }

    static {
      kAlgaeRollerMotorConfiguration.currentLimit = 30;
      kAlgaeRollerMotorConfiguration.gearRatio = kCoralRollerGearRatio;
      kAlgaeRollerMotorConfiguration.isInverted = false;
      kAlgaeRollerMotorConfiguration.mode = MotorMode.kFlywheel;
      kAlgaeRollerMotorConfiguration.idleState = IdleState.kCoast;
      kAlgaeRollerMotorConfiguration.finalDiameterMeters = kAlgaeWheelDiameter;

      kPivotMotorConfiguration.withFOC = false;

      kAlgaeRollerMotorConfiguration.maxVelocity =
          kAlgaeRollerMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKraken_MaxRPM);
      kAlgaeRollerMotorConfiguration.maxAcceleration =
          kAlgaeRollerMotorConfiguration.maxVelocity * 2.0;

      kAlgaeRollerMotorConfiguration.altA = kAlgaeRollerMotorConfiguration.maxAcceleration;
      kAlgaeRollerMotorConfiguration.altV = kAlgaeRollerMotorConfiguration.maxVelocity;

      kAlgaeRollerMotorConfiguration.kP = 0.1;
      kAlgaeRollerMotorConfiguration.kA = 0.1;
      kAlgaeRollerMotorConfiguration.kV = 0.1;
      kAlgaeRollerMotorConfiguration.kD = 0.1;

      kCoralRollerMotorConfiguration.tolerance = 0.05;
    }

    static {
      kSIMPivotController.setTolerance(kPivotMotorConfiguration.tolerance);
      kSIMWristController.setTolerance(kWristMotorConfiguration.tolerance);
    }
  }

  public static class VisionConstants {
    public static final String swerveCamName = "swerveCam";
    public static final String intakeCamName = "intakeCam";
    public static final String backCamName = "backCam";

    // public static final Transform3d swerveCamTransform = new Transform3d(kLoopSpeed,
    // kOdometryFrequency, kLoopSpeed, null)
  }
}
