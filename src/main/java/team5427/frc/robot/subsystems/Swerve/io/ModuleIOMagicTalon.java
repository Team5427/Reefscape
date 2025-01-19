package team5427.frc.robot.subsystems.Swerve.io;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.PhoenixOdometryThread;
import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.SteelTalonFX;

public class ModuleIOMagicTalon implements ModuleIO {

    private MagicSteelTalonFX steerMotor;
    private MagicSteelTalonFX driveMotor;

    private CANcoder cancoder;

    private final int moduleIdx;

    private SwerveModuleState targetModuleState;
    private Voltage steerMotorVoltage = Volts.of(0.0);
    private Voltage driveMotorVoltage = Volts.of(0.0);

    private Queue<Double> drivePositionQueue;
    private Queue<Double> steerPositionQueue;

    private final Queue<Double> timestampQueue;

    public ModuleIOMagicTalon(int moduleIdx) {

        this.moduleIdx = moduleIdx;

        driveMotor = new MagicSteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[moduleIdx]);
        steerMotor = new MagicSteelTalonFX(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[moduleIdx]);
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
        driveMotorVoltage = driveMotor.getTalonFX().getMotorVoltage().getValue();
        steerMotorVoltage = steerMotor.getTalonFX().getMotorVoltage().getValue();

        inputs.absolutePosition = getCancoderRotation();

        inputs.currentModuleState = new SwerveModuleState(
                driveMotor.getEncoderVelocity(),
                inputs.absolutePosition);
        inputs.driveMotorPosition = Rotation2d.fromRotations(driveMotor.getEncoderPosition());
        inputs.steerMotorVelocityRotations = RotationsPerSecond.of(steerMotor.getEncoderVelocity()/60.0);
        inputs.targetModuleState = targetModuleState;

        inputs.steerPosition = Rotation2d.fromRotations(steerMotor.getEncoderPosition());

        inputs.currentModulePosition = new SwerveModulePosition(
                driveMotor.getEncoderPosition(),
                inputs.absolutePosition);
        inputs.driveMotorVoltage = driveMotorVoltage;
        inputs.steerMotorVoltage = steerMotorVoltage;

        inputs.driveMotorConnected = driveMotor.getTalonFX().isConnected();
        inputs.steerMotorConnected = steerMotor.getTalonFX().isConnected();

        inputs.driveMotorCurrent = driveMotor.getTalonFX().getStatorCurrent().getValue();
        inputs.steerMotorCurrent = steerMotor.getTalonFX().getStatorCurrent().getValue();

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
    public void setDriveSpeedSetpoint(LinearVelocity speed) {
        driveMotor.setSetpoint(speed.baseUnitMagnitude());
        driveMotorVoltage = driveMotor.getTalonFX().getMotorVoltage().getValue();
    }

    /**
     * @param setpoint - <strong>Rotation2d</strong> setpoint for steer motor
     *                 (module angle)
     **/
    @Override
    public void setSteerPositionSetpoint(Rotation2d setpoint) {
        steerMotor.setSetpoint(setpoint);
        steerMotorVoltage = steerMotor.getTalonFX().getMotorVoltage().getValue();
    }

    @Override
    public void setModuleState(SwerveModuleState state) {
        state.optimize(getCancoderRotation());
        targetModuleState = state;
        setDriveSpeedSetpoint(MetersPerSecond.of(targetModuleState.speedMetersPerSecond));
        setSteerPositionSetpoint(targetModuleState.angle);
    }
}