package team5427.lib.motors.simulated;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import team5427.frc.robot.RobotContainer;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.IMotorController;
import team5427.lib.motors.real.MotorConfiguration;
import team5427.lib.motors.simulated.SimMotorConfiguration.SimMotorMode;

public class SimSteelTalonFX implements IMotorController {

    private CANDeviceId id;
    private TalonFX talonFX;
    private SimMotorConfiguration configuration;

    private TalonFXSimState simState;

    private double positionConversionFactor;
    private double velocityConversionFactor;

    private double setpoint;

    private boolean withFOC;

    public SimSteelTalonFX(CANDeviceId id) {
        this.id = id;

        talonFX = new TalonFX(this.id.getDeviceNumber());

        withFOC = false;
        simState = talonFX.getSimState();
    }

    public void apply(SimMotorConfiguration configuration) {
        if (id.getBus() != "") {
            throw new Error(String.format(
                    "Error Simulated Motor with ID %d cannot have a bus name and needs to have a unique ID",
                    id.getDeviceNumber()));
        }
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        positionConversionFactor = configuration.unitConversionRatio;
        velocityConversionFactor = configuration.unitConversionRatio / 60.0;

        talonConfig.Feedback.SensorToMechanismRatio = configuration.gearRatio.getSensorToMechanismRatio();

        talonConfig.Slot0.kP = configuration.kP;
        talonConfig.Slot0.kI = configuration.kI;
        talonConfig.Slot0.kD = configuration.kD;
        talonConfig.Slot0.kS = configuration.kS;
        talonConfig.Slot0.kV = configuration.kV;
        talonConfig.Slot0.kA = configuration.kA;

        talonConfig.ClosedLoopGeneral.ContinuousWrap = true;

        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFX.getConfigurator().apply(talonConfig);
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        simState.Orientation = configuration.orientation;
        this.configuration = configuration;
    }

    @Override
    public void setSetpoint(double setpoint) {
        switch (configuration.mode) {
            case kFlywheel:
                talonFX.setControl(new VelocityVoltage(setpoint / velocityConversionFactor).withEnableFOC(withFOC));
                break;
            case kServo:
            case kLinear:
                talonFX.setControl(new PositionVoltage(setpoint / positionConversionFactor).withEnableFOC(withFOC));
                break;
            default:
                talonFX.setControl(new VelocityVoltage(setpoint / velocityConversionFactor).withEnableFOC(withFOC));
                break;
        }
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public void setEncoderPosition(double position) {
        talonFX.setPosition(position);
    }

    @Override
    public double getEncoderPosition() {
        return talonFX.getPosition().getValueAsDouble() * positionConversionFactor;
    }

    @Override
    public double getEncoderVelocity() {
        return talonFX.getVelocity().getValueAsDouble() * velocityConversionFactor;
    }

    @Override
    public void setRawPercentage(double percentage) {
        talonFX.set(percentage);
    }

    @Override
    public void setRelativePercentage(double percentage) {
        talonFX.setVoltage(percentage * talonFX.getSupplyVoltage().getValueAsDouble());
    }

    @Override
    public double getError() {
        if (configuration.mode == SimMotorMode.kFlywheel) {
            return setpoint - getEncoderVelocity();
        }
        return setpoint - getEncoderPosition();
    }

    public void setFOC(boolean foc) {
        withFOC = foc;
    }

    public TalonFX getTalonFX() {
        return talonFX;
    }

    public TalonFXSimState getSimState() {
        return simState;
    }

    @Override
    public void apply(MotorConfiguration configuration) {
        throw new Error(
                "Simulated Motor cannot use a MotorConfiguration.\nSimulated Motors need to use a SimulatedMotorConfiguration");
    }

    @Override
    public void setSetpoint(Rotation2d setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setEncoderPosition'");
    }

    @Override
    public void setRawVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRawVoltage'");
    }

}
