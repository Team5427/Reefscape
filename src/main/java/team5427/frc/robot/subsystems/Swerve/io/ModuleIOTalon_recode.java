package team5427.frc.robot.subsystems.Swerve.io;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.PhoenixOdometryThread;
import team5427.lib.motors.real.SteelTalonFX;

public class ModuleIOTalon_recode implements ModuleIO {

    private final int moduleIdx;

    private SteelTalonFX driveTalon;
    private SteelTalonFX steerTalon;

    private CANcoder cancoder;

    private SwerveModuleState targetModuleState;
    private StatusSignal<Voltage> driveVoltage;
    private StatusSignal<Voltage> steerVoltage;

    private StatusSignal<Angle> steerPosition;
    private StatusSignal<AngularVelocity> steerVelocity;
    private StatusSignal<Angle> absolutePosition;
    private StatusSignal<Velocity> driveVelocity;
    private StatusSignal<Angle> drivePosition;

    private Queue<Double> drivePositionQueue;
    private Queue<Double> steerPositionQueue;

    private final Queue<Double> timestampQueue;

    public ModuleIOTalon_recode(int moduleIdx) {

        this.moduleIdx = moduleIdx;

        driveTalon = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[moduleIdx]);
        steerTalon = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[moduleIdx]);

        cancoder = new CANcoder(
            SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getDeviceNumber(),
            SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getBus()
        );

        SwerveConstants.kDriveMotorConfiguration.isInverted = SwerveConstants.kSwerveUtilInstance.kDriveInversion[moduleIdx];
        SwerveConstants.kSteerMotorConfiguration.isInverted = SwerveConstants.kSwerveUtilInstance.kSteerInversion[moduleIdx];

        driveTalon.apply(SwerveConstants.kDriveMotorConfiguration);
        steerTalon.apply(SwerveConstants.kSteerMotorConfiguration);

        driveTalon.getTalonFX().clearStickyFaults();
        steerTalon.getTalonFX().clearStickyFaults();

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfig.MagnetSensor.MagnetOffset = SwerveConstants.kSwerveUtilInstance.kModuleOffsets[moduleIdx];
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        cancoder.clearStickyFaults();

        steerTalon.talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        steerTalon.talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerTalon.talonConfig.Feedback.SensorToMechanismRatio = 1.0;
        steerTalon.talonConfig.Feedback.RotorToSensorRatio = SwerveConstants.kSteerMotorConfiguration.gearRatio.getMathematicalGearRatio();
        steerTalon.getTalonFX().getConfigurator().apply(steerTalon.talonConfig);

        steerTalon.setEncoderPosition(steerPosition.getValue().in(Rotations));
        driveTalon.setEncoderPosition(0.0);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getTalonFX().getPosition());
        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getTalonFX().getPosition());

        ParentDevice.optimizeBusUtilizationForAll(driveTalon.getTalonFX(), steerTalon.getTalonFX(), cancoder);

        System.out.println("Created a new module of idx: " + moduleIdx);

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        
        inputs.absolutePosition = Rotation2d.fromRotations(absolutePosition.getValue().in(Rotations));
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValue().in(Rotations));
        inputs.currentModuleState = new SwerveModuleState(driveVelocity.getValueAsDouble(), inputs.absolutePosition);
        inputs.targetModuleState = targetModuleState;

        inputs.currentModulePosition = new SwerveModulePosition(drivePosition.getValueAsDouble(), inputs.absolutePosition);
        inputs.driveMotorPosition = Rotation2d.fromRotations(drivePosition.getValue().in(Rotations));
        inputs.steerMotorVelocityRotations = steerVelocity.getValue();
    }

    
    
}
