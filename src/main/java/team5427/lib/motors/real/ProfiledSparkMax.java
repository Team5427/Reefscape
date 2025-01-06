package team5427.lib.motors.real;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.IMotorController;
import team5427.lib.motors.real.MotorConfiguration.IdleState;
import team5427.lib.motors.real.MotorConfiguration.MotorMode;

public class ProfiledSparkMax implements IMotorController {

    private CANDeviceId id;
    private SparkMax sparkMax;
    private MotorConfiguration configuration;
    private RelativeEncoder relativeEncoder;
    private SparkMaxConfig config;
    private ProfiledPIDController controller;

    private double setpoint;

    public ProfiledSparkMax(CANDeviceId id) {
        this.id = id;

        sparkMax = new SparkMax(this.id.getDeviceNumber(), MotorType.kBrushless);

        relativeEncoder = sparkMax.getEncoder();
        // sparkMax.getClosedLoopController()
        // relativeEncoder.setMeasurementPeriod(10);
        config = new SparkMaxConfig();
        controller = new ProfiledPIDController(0, 0, 0, null);
    }

    @Override
    public void apply(MotorConfiguration configuration) {
        this.configuration = configuration;

        config.inverted(configuration.isInverted)
                .idleMode(configuration.idleState == IdleState.kBrake ? IdleMode.kBrake : IdleMode.kCoast);

        // sparkMax.setInverted(configuration.isInverted);
        // sparkMax.setIdleMode(configuration.idleState == IdleState.kBrake ?
        // IdleMode.kBrake : IdleMode.kCoast);
        config.encoder.positionConversionFactor(configuration.unitConversionRatio)
                .velocityConversionFactor(configuration.unitConversionRatio / 60.0);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // relativeEncoder.setPositionConversionFactor(configuration.unitConversionRatio);
        // relativeEncoder.setVelocityConversionFactor(configuration.unitConversionRatio
        // / 60.0);

        config.closedLoop.pid(configuration.kP, configuration.kI, configuration.kD);
        config.closedLoop.maxMotion.maxAcceleration(configuration.maxAcceleration)
                .maxVelocity(configuration.maxVelocity).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        config.closedLoop.positionWrappingEnabled(true).positionWrappingMinInput(-Math.PI)
                .positionWrappingMaxInput(Math.PI);
        // controller.setP(configuration.kP);
        // controller.setI(configuration.kI);
        // controller.setD(configuration.kD);
        // controller.setConstraints(new TrapezoidProfile.Constraints(
        // configuration.maxVelocity, configuration.maxAcceleration));

        // controller.enableContinuousInput(-Math.PI, Math.PI);

        if (configuration.mode == MotorMode.kFlywheel) {
            throw new Error("Profiled Spark Max of id " + id.getDeviceNumber() + " is set as an illegal flywheel.");
        }
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // sparkMax.burnFlash();
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        sparkMax.setVoltage(
                controller.calculate(getEncoderPosition(), this.setpoint) + configuration.kFF * this.setpoint);
    }

    public void setSetpoint(Rotation2d setpoint) {
        this.setpoint = setpoint.getRadians();
        sparkMax.setVoltage(
                controller.calculate(getEncoderPosition(), this.setpoint) + configuration.kFF * this.setpoint);
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public void setEncoderPosition(double position) {
        relativeEncoder.setPosition(position);
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
        relativeEncoder.setPosition(position.getRotations());
    }

    @Override
    public double getEncoderPosition() {
        return relativeEncoder.getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return relativeEncoder.getVelocity();
    }

    @Override
    public void setRawPercentage(double percentage) {
        sparkMax.set(percentage);
    }

    @Override
    public void setRelativePercentage(double percentage) {
        sparkMax.setVoltage(percentage * sparkMax.getBusVoltage());
    }

    @Override
    public void setRawVoltage(double voltage) {
        sparkMax.setVoltage(voltage);
    }

    @Override
    public double getError() {
        if (configuration.mode == MotorMode.kFlywheel) {
            return setpoint - getEncoderVelocity();
        }
        return setpoint - getEncoderPosition();
    }

    public SparkMax getSparkMax() {
        return sparkMax;
    }

    public RelativeEncoder getRelativeEncoder() {
        return relativeEncoder;
    }

    public MotorConfiguration getConfiguration() {
        return configuration;
    }

}
