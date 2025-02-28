package team5427.frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
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
  public static final String kCanivoreBusName = "brook";
  public static final double kOdometryFrequency =
      new CANBus(Constants.kCanivoreBusName).isNetworkFD() ? 250.0 : 100.0;
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
    public static final double kDriverControllerJoystickDeadzone = 0.0;
  }

  public static class SwerveConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.9);
    public static final double kTrackWidth = Units.inchesToMeters(22.75);
    public static final double kWheelBase = Units.inchesToMeters(22.75);

    public static final PIDController kSIMSteerController = new PIDController(5, 0, 0.2);
    public static final SimpleMotorFeedforward kSIMSteerFeedforward =
        new SimpleMotorFeedforward(0, 0.01, 0);

    public static final PIDController kSIMDriveController = new PIDController(4, 0, 0.6);
    public static final SimpleMotorFeedforward kSIMDriveFeedforward =
        new SimpleMotorFeedforward(0., 2.08, 0.17);

    public static final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveUtil kSwerveUtilInstance = new SwerveUtil();

    static {
      kSIMSteerController.enableContinuousInput(-0.5, 0.5);
      kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(3, "*");
      kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(5, "*");
      kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(7, "*");
      kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(9, "*");

      kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(4, "*");
      kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(6, "*");
      kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(8, "*");
      kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(10, "*");

      kSwerveUtilInstance.kCancoderIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(12, "*");
      kSwerveUtilInstance.kCancoderIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(13, "*");
      kSwerveUtilInstance.kCancoderIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(14, "*");
      kSwerveUtilInstance.kCancoderIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(15, "*");

      kSwerveUtilInstance.kDriveInversion[SwerveUtil.kFrontLeftModuleIdx] = true;
      kSwerveUtilInstance.kDriveInversion[SwerveUtil.kFrontRightModuleIdx] = true;
      kSwerveUtilInstance.kDriveInversion[SwerveUtil.kRearLeftModuleIdx] = true;
      kSwerveUtilInstance.kDriveInversion[SwerveUtil.kRearRightModuleIdx] = true;

      kSwerveUtilInstance.kSteerInversion[SwerveUtil.kFrontLeftModuleIdx] = false;
      kSwerveUtilInstance.kSteerInversion[SwerveUtil.kFrontRightModuleIdx] = false;
      kSwerveUtilInstance.kSteerInversion[SwerveUtil.kRearLeftModuleIdx] = false;
      kSwerveUtilInstance.kSteerInversion[SwerveUtil.kRearRightModuleIdx] = false;

      kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kFrontLeftModuleIdx] = 0.20752;

      kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kFrontRightModuleIdx] = -0.407;

      kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kRearRightModuleIdx] = -0.2105;

      kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kRearLeftModuleIdx] = -0.1358;

      //  kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kFrontLeftModuleIdx] = 0.;

      // kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kFrontRightModuleIdx] = -0.;

      // kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kRearRightModuleIdx] = -0.;

      // kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kRearLeftModuleIdx] = -0.;
    }

    public static MotorConfiguration kDriveMotorConfiguration = new MotorConfiguration();

    static {
      kDriveMotorConfiguration.gearRatio = SwerveUtil.kSDSL3GearRatio;
      kDriveMotorConfiguration.idleState = IdleState.kBrake;
      kDriveMotorConfiguration.mode = MotorMode.kFlywheel;
      kDriveMotorConfiguration.withFOC = true;

      kDriveMotorConfiguration.currentLimit = 80;
      kDriveMotorConfiguration.finalDiameterMeters = kWheelDiameterMeters;

      kDriveMotorConfiguration.maxVelocity =
          kDriveMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM);
      kDriveMotorConfiguration.maxAcceleration = kDriveMotorConfiguration.maxVelocity * 2.0;

      kDriveMotorConfiguration.kP = 2.5;
      // kDriveMotorConfiguration.kV = 2.08;
      kDriveMotorConfiguration.kA = 0.5;
      kDriveMotorConfiguration.kS = 0.23;
      kDriveMotorConfiguration.altV = kDriveMotorConfiguration.maxVelocity;
      kDriveMotorConfiguration.altA = kDriveMotorConfiguration.maxAcceleration;
    }

    public static MotorConfiguration kSteerMotorConfiguration = new MotorConfiguration();

    static {
      kSteerMotorConfiguration.gearRatio = SwerveUtil.kSDSSteerGearRatioMK4n;
      kSteerMotorConfiguration.idleState = IdleState.kBrake;
      kSteerMotorConfiguration.mode = MotorMode.kServo;
      kSteerMotorConfiguration.currentLimit = 30;
      kSteerMotorConfiguration.withFOC = true;

      kSteerMotorConfiguration.maxVelocity =
          kSteerMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM);
      kSteerMotorConfiguration.maxAcceleration = kSteerMotorConfiguration.maxVelocity * 1000.0;

      // Tunable values
      kSteerMotorConfiguration.kP = 6.7; // 7.0
      kSteerMotorConfiguration.kD = 0.18;
      kSteerMotorConfiguration.kS = 0.21;
      kSteerMotorConfiguration.kA = 0.1;
      // kSteerMotorConfiguration.kA = 8.0;
      // kSteerMotorConfiguration.altV = kSteerMotorConfiguration.maxVelocity;
      // kSteerMotorConfiguration.altA = kSteerMotorConfiguration.maxAcceleration;
    }

    public static final CANDeviceId kPigeonCANId = new CANDeviceId(11, "*");

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

    public static final Rotation2d kWristMinimumAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d kWristMaximumAngle = Rotation2d.fromDegrees(180);
    public static final Rotation2d kPivotMinimumAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d kPivotMaximumAngle = Rotation2d.fromDegrees(180);

    public static final Rotation2d kPivotBufferAngle = Rotation2d.fromDegrees(10);

    public static final ComplexGearRatio kWristGearRatio = new ComplexGearRatio((1.0));
    public static final ComplexGearRatio kPivotGearRatio = new ComplexGearRatio((14.0/70.0), (18.0/72.0), (15.0 / 36.0));
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
    }

    static {
      kSIMPivotController.setTolerance(0.05);
      kSIMWristController.setTolerance(0.05);
    }
  }

  public static class VisionConstants {
    public static final String kSwerveCamName = "swerveCam";
    // public static final String intakeCamName = "intakeCam";
    public static final String kBackCamName = "backCam";

    public static final int kCameraCount = 2;

    public static final double kMaxAmbiguity = 0.20;

    public static final Distance kMaxZHeight = Meters.of(0.5);

    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final Transform3d kSwerveCamTransform =
        new Transform3d(0, 0, 0, Rotation3d.kZero);
    public static final Transform3d kBackCamTransform = new Transform3d(0, 0, 0, Rotation3d.kZero);
    public static final Distance kCameraMaxRange = Distance.ofBaseUnits(4.0, Meters);

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    /** Larger stddev equals more doubt in Meters */
    public static double kLinearStdDevBaseline = 0.02;

    /** Larger stddev equals more doubt in Radians */
    public static double kAngularStdDevBaseline = 0.06;

    public static double[] kCameraStdDevFactors =
        new double[] {
          1.0, // Swerve Cam
          1.0 // Back Cam
        };
  }

  public static class CascadeConstants {
    public static final CANDeviceId kCascadeMasterId = new CANDeviceId(20);
    public static final CANDeviceId kCascadeSlaveId = new CANDeviceId(21);

    // public static final double kCascadeDriverGravityFF = 0.0;

    public static final Rotation2d kCascadePivotBuffer = Rotation2d.fromDegrees(30);
    public static final Debouncer kCascadePivotDebouncer = new Debouncer(0.25, DebounceType.kBoth);

    public static final Rotation2d kCascadePivotMaximumAngle = Rotation2d.fromDegrees(30);
    public static final Rotation2d kCascadePivotMinimumAngle = Rotation2d.fromDegrees(-30);

    public static final Distance kCascadeMinimumHeight = Meters.of(0.0);
    public static final Distance kCascadeMaximumHeight = Meters.of(2.0);

    public static final MotorConfiguration kCascadeDriverConfiguration = new MotorConfiguration();

    static {
      kCascadeDriverConfiguration.gearRatio = new ComplexGearRatio(20.0 / 66.0);
      kCascadeDriverConfiguration.currentLimit = 60;
      kCascadeDriverConfiguration.idleState = IdleState.kBrake;
      kCascadeDriverConfiguration.mode = MotorMode.kLinear;
      kCascadeDriverConfiguration.isInverted = true;
      kCascadeDriverConfiguration.withFOC = true;

      kCascadeDriverConfiguration.maxVelocity =
          kCascadeDriverConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM) * 0.5;
      kCascadeDriverConfiguration.maxAcceleration = kCascadeDriverConfiguration.maxVelocity * 2.0;

      kCascadeDriverConfiguration.finalDiameterMeters = Units.inchesToMeters(1.4875);

      kCascadeDriverConfiguration.kP = .155;
      // kCascadeDriverConfiguration.kI = .2;
      // kCascadeDriverConfiguration.kG = 0.36;
      kCascadeDriverConfiguration.kD = 0.02;
      // kCascadeDriverConfiguration.kV = 3.0;
      // kCascadeDriverConfiguration.kV = .50;
      // kCascadeDriverConfiguration.kA = 0.05;
      // kCascadeDriverConfiguration.kS = 0.1;
      // kCascadeDriverConfiguration.kFF = 0.0125;

      kCascadeDriverConfiguration.altA = kCascadeDriverConfiguration.maxAcceleration;
      kCascadeDriverConfiguration.altV = kCascadeDriverConfiguration.maxVelocity;
      kCascadeDriverConfiguration.altJ = 3000.0000000000001;
    }

    public static final CANDeviceId kPivotMasterId = new CANDeviceId(16, "*");
    public static final CANDeviceId kPivotSlaveId = new CANDeviceId(17, "*");

    public static final MotorConfiguration kPivotConfiguration = new MotorConfiguration();

    static {
      kPivotConfiguration.gearRatio =
          new ComplexGearRatio((1.0 / 5.0), (1.0 / 3.0), (1.0 / 3.0), (32.0 / 48.0), (9.0 / 44.0));
      kPivotConfiguration.currentLimit = 60;
      kPivotConfiguration.idleState = IdleState.kBrake;
      kPivotConfiguration.mode = MotorMode.kServo;
      kPivotConfiguration.isInverted = true;

      kPivotConfiguration.maxVelocity =
          kPivotConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM);
      kPivotConfiguration.maxAcceleration = kPivotConfiguration.maxVelocity / 2.0;

      kPivotConfiguration.kP = 12.0;
      kPivotConfiguration.kD = 0.0;
      // kPivotConfiguration.kV = 22.76;
      // kPivotConfiguration.kA = 0.19;
      // kPivotConfiguration.kS = 0.0;
      // kPivotConfiguration.kG = 0.32;

      kPivotConfiguration.altA = kPivotConfiguration.maxAcceleration;
      kPivotConfiguration.altV = kPivotConfiguration.maxVelocity;

    }

    public static final CANDeviceId kPivotCANcoderId = new CANDeviceId(18, "*");

    public static final Distance kCascadeTolerance = Centimeters.of(1.0);
    public static final Rotation2d kPivotTolerance = Rotation2d.fromDegrees(0.5);

    public static final double kPivotCancoderOffset = 0.128;

    public static final Distance kStowDistance = Feet.of(0.25);
    public static final Distance kL1Distance = Feet.of(0.5);
    public static final Distance kL2Distance = Feet.of(1.0);
    public static final Distance kL3Distance = Feet.of(2.5);
    public static final Distance kL4Distance = Feet.of(3.5);

    public static final Rotation2d kStowRotation = Rotation2d.kZero;
    public static final Rotation2d kTempActiveRotation = Rotation2d.fromDegrees(30.0);
    public static final Rotation2d kTempClimbRotation = Rotation2d.fromDegrees(60.0);
  }

  public static class ClimbConstants {
    public static final CANDeviceId kHookServoId = new CANDeviceId(0);

    public static final MotorConfiguration kServoConfiguration = new MotorConfiguration();
    static {
      kServoConfiguration.gearRatio = new ComplexGearRatio(1.0 / 3.0);
      kServoConfiguration.idleState = IdleState.kBrake;
      kServoConfiguration.isInverted = false;
      kServoConfiguration.mode = MotorMode.kServo;
      kServoConfiguration.withFOC = true;
      
      kServoConfiguration.maxVelocity = kServoConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenFOC_MaxRPM);
      kServoConfiguration.maxAcceleration = kServoConfiguration.maxVelocity * 3.0;

      kServoConfiguration.kP = 0.5;
    }
  }
}

