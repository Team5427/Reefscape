package team5427.frc.robot.subsystems.Swerve.io;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.PhoenixOdometryThread;
import team5427.lib.motors.real.SteelTalonFX;

public class ModuleIOTalon implements ModuleIO {

    private SteelTalonFX steerMotor;
    private SteelTalonFX driveMotor;

    private CANcoder cancoder;

    private final int moduleIdx;

    private SwerveModuleState targetModuleState;
    private double steerMotorVoltage = 0;
    private double driveMotorVoltage = 0;

    private Queue<Double> drivePositionQueue;
    private Queue<Double> steerPositionQueue;

    private final Queue<Double> timestampQueue;

    public ModuleIOTalon(int moduleIdx) {

        this.moduleIdx = moduleIdx;

        driveMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[moduleIdx]);
        steerMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[moduleIdx]);
        cancoder = new CANcoder(SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getDeviceNumber(),
                SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getBus());
        SwerveConstants.kDriveMotorConfiguration.isInverted = SwerveConstants.kSwerveUtilInstance.kDriveInversion[moduleIdx];
        SwerveConstants.kSteerMotorConfiguration.isInverted = SwerveConstants.kSwerveUtilInstance.kSteerInversion[moduleIdx];
        driveMotor.apply(SwerveConstants.kDriveMotorConfiguration);
        steerMotor.apply(SwerveConstants.kSteerMotorConfiguration);

        driveMotor.getTalonFX().clearStickyFaults();
        steerMotor.getTalonFX().clearStickyFaults();

        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);
        configuration.MagnetSensor.MagnetOffset = SwerveConstants.kSwerveUtilInstance.kModuleOffsets[moduleIdx];
        configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(configuration);

        cancoder.clearStickyFaults();

        steerMotor.setEncoderPosition(cancoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI);
        driveMotor.setEncoderPosition(0.0);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getTalonFX().getPosition());

        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerMotor.getTalonFX().getPosition());

        ParentDevice.optimizeBusUtilizationForAll(driveMotor.getTalonFX(), steerMotor.getTalonFX());

        System.out.println("New Module with idx: " + moduleIdx);

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveMotorVoltage = driveMotor.getTalonFX().getMotorVoltage().getValueAsDouble();
        steerMotorVoltage = steerMotor.getTalonFX().getMotorVoltage().getValueAsDouble();

        inputs.absolutePosition = getCancoderRotation();

        inputs.currentModuleState = new SwerveModuleState(
                driveMotor.getEncoderVelocity(),
                inputs.absolutePosition);
        inputs.driveMotorPosition = Rotation2d.fromRotations(driveMotor.getEncoderPosition());
        inputs.steerMotorVelocityRotationsPerSecond = steerMotor.getEncoderVelocity();
        inputs.targetModuleState = targetModuleState;

        inputs.steerPosition = Rotation2d.fromRotations(steerMotor.getEncoderPosition());

        inputs.currentModulePosition = new SwerveModulePosition(
                driveMotor.getEncoderPosition(),
                inputs.absolutePosition);
        inputs.driveMotorVoltage = driveMotorVoltage;
        inputs.steerMotorVoltage = steerMotorVoltage;

        inputs.driveMotorConnected = driveMotor.getTalonFX().isConnected();
        inputs.steerMotorConnected = steerMotor.getTalonFX().isConnected();

        inputs.driveMotorCurrent = driveMotor.getTalonFX().getStatorCurrent().getValueAsDouble();
        inputs.steerMotorCurrent = steerMotor.getTalonFX().getStatorCurrent().getValueAsDouble();

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsMeters = drivePositionQueue.stream()
                // converts the raw rotations -> radians -> meters
                .mapToDouble((Double value) -> value * Math.PI * SwerveConstants.kWheelDiameterMeters)
                .toArray();
        inputs.odometryTurnPositions = steerPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();

    }

    private Rotation2d getCancoderRotation() {

        return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void resetMotorSetpoint() {
        steerMotor.setEncoderPosition(getCancoderRotation());
        driveMotor.setEncoderPosition(0.0);
    }

    /**
     * Needs to be a <strong>Double</strong>, NOT a double
     * 
     * @param speed - <strong>Double</strong>, setpoint for drive motor (module
     *              speed)
     **/
    @Override
    public void setDriveSpeedSetpoint(Double speed) {
        driveMotor.setSetpoint(Rotation2d.fromRotations(speed));
        driveMotorVoltage = driveMotor.getTalonFX().getMotorVoltage().getValueAsDouble();
    }

    /**
     * @param setpoint - <strong>Rotation2d</strong> setpoint for steer motor
     *                 (module angle)
     **/
    @Override
    public void setSteerPositionSetpoint(Rotation2d setpoint) {
        steerMotor.setSetpoint(setpoint);
        steerMotorVoltage = steerMotor.getTalonFX().getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setModuleState(SwerveModuleState state) {
        targetModuleState = state;
        setDriveSpeedSetpoint(Double.valueOf(targetModuleState.speedMetersPerSecond));
        setSteerPositionSetpoint(targetModuleState.angle);
    }
}