package team5427.lib.motors.real;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.IMotorController;
import team5427.lib.motors.real.MotorConfiguration.MotorMode;


public class SteelTalonFX implements IMotorController {

    private CANDeviceId id;
    private TalonFX talonFX;
    private MotorConfiguration configuration;


    private double positionConversionFactor;
    // This is Radians/Second
    private double velocityConversionFactor;

    private double setpoint;

    private boolean withFOC;

    public SteelTalonFX(CANDeviceId id) {
        this.id = id;

        talonFX = new TalonFX(this.id.getDeviceNumber());

        withFOC = false;
    }

    @Override
    public void apply(MotorConfiguration configuration) {
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

        switch(configuration.mode){
            case kFlywheel:
                talonConfig.ClosedLoopGeneral.ContinuousWrap = false;
                break;
            case kServo:
            case kLinear:
                talonConfig.ClosedLoopGeneral.ContinuousWrap = true;
                break;
            default:
                break;
            
        }

        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.StatorCurrentLimit = configuration.currentLimit;
        talonConfig.CurrentLimits.SupplyCurrentLimit = configuration.currentLimit;

        talonFX.getConfigurator().apply(talonConfig);
    }

    // /**
    //  * @param setpoint - Rotation2d
    //  **/
    // @Override
    // public void setSetpoint(Rotation2d setpoint) {
    //     switch (configuration.mode) {
    //         case kFlywheel:
    //             talonFX.setControl(new VelocityVoltage(setpoint.getRotations()).withEnableFOC(withFOC));
    //             break;
    //         case kServo:
    //         case kLinear:
    //             talonFX.setControl(new PositionVoltage(setpoint.getRotations()).withEnableFOC(withFOC));
    //             break;
    //         default:
    //             talonFX.setControl(new VelocityVoltage(setpoint.getRotations()).withEnableFOC(withFOC));
    //             break;
    //     }
    // }
        /**
     * @param setpoint - Rotation2d
     **/
    @Override
    public void setSetpoint(Rotation2d setpoint) {
        switch (configuration.mode) {
            case kFlywheel:
                talonFX.setControl(
                        new VelocityVoltage(setpoint.getRotations()).withEnableFOC(withFOC));
                break;
            case kServo:
            case kLinear:
                talonFX.setControl(new PositionDutyCycle(setpoint.getRotations()).withEnableFOC(withFOC));
                break;
            default:
                talonFX.setControl(new VelocityVoltage(setpoint.getRotations() * velocityConversionFactor).withEnableFOC(withFOC));
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
    public void setEncoderPosition(Rotation2d position) {
        talonFX.setPosition(position.getRotations());
    }

    /**
     * In radians
     */
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
    public void setRawVoltage(double voltage) {
        talonFX.setVoltage(voltage);
    }

    @Override
    public double getError() {
        if (configuration.mode == MotorMode.kFlywheel) {
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

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        switch (configuration.mode) {
            // maybe its * instead of /
            case kFlywheel:
                talonFX.setControl(
                        new VelocityVoltage(setpoint / velocityConversionFactor).withEnableFOC(withFOC));
                break;
            case kServo:
            case kLinear:
                talonFX.setControl(new PositionDutyCycle(setpoint / positionConversionFactor).withEnableFOC(withFOC));
                break;
            default:
                talonFX.setControl(new VelocityVoltage(setpoint / velocityConversionFactor).withEnableFOC(withFOC));
                break;
        }
    }
    // /**
    //  * 
    //  @param setpoint - Radians setpoint
    //  */
    // @Override
    // public void setSetpoint(double setpoint) {
    //     setSetpoint(new Rotation2d(setpoint));
    // }


}
