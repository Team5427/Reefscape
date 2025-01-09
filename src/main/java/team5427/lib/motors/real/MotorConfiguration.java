package team5427.lib.motors.real;

import com.google.errorprone.annotations.DoNotCall;

import team5427.lib.drivers.ComplexGearRatio;

public class MotorConfiguration {

    public static enum MotorMode {
        kFlywheel,
        kServo,
        kLinear
    }

    public static enum IdleState {
        kBrake,
        kCoast,
    }

    public boolean isInverted;

    public double kP, kI, kD, kFF;
    public double kS, kV, kA;

    public int currentLimit;

    public ComplexGearRatio gearRatio;
    public double finalDiameterMeters;

    /**
     * Converts between Radians, Degrees, Rotations time units etc.
     * @deprecated will no longer be used as rotations are the default
     */
    @Deprecated
    public double unitConversionRatio;
    
    public double maxVelocity, maxAcceleration;

    public IdleState idleState;

    public MotorMode mode;

    public MotorConfiguration() {
        isInverted = false;

        kP = kI = kD = kFF = 0.0;
        kS = kV = kA = 0.0;

        currentLimit = 30;

        gearRatio = new ComplexGearRatio();
        finalDiameterMeters = 1.0;
        unitConversionRatio = 1.0;
        maxVelocity = 1.0;
        maxAcceleration = 1.0;

        idleState = IdleState.kBrake;
        mode = MotorMode.kFlywheel;
    }

    public MotorConfiguration(MotorConfiguration parent) {
        isInverted = parent.isInverted;

        kP = parent.kP;
        kI = parent.kI;
        kD = parent.kD;
        kFF = parent.kFF;

        kS = parent.kS;
        kV = parent.kV;
        kA = parent.kA;

        currentLimit = parent.currentLimit;

        gearRatio = parent.gearRatio;
        unitConversionRatio = parent.unitConversionRatio;
        finalDiameterMeters = parent.finalDiameterMeters;

        maxVelocity = parent.maxVelocity;
        maxAcceleration = parent.maxAcceleration;

        idleState = parent.idleState;
        mode = parent.mode;
    }
    
    // /**
    //  * @return Radians Per Second
    //  * @deprecated no longer 
    //  */
    // @Deprecated(since="1/7/2025", forRemoval = true)
    // public double getStandardMaxVelocity(double maxMotorRPM) {
    //     return (maxMotorRPM * getStandardUnitConversionRatio()) / 60.0;
    // }

    /**
     * 
     * @param maxMotorRPM - maximum motor speed in rotations per minute
     * @return if servo, then the maximum rotations per second of the motor.
     * If a flywheel or linear, then the meters per second of the motor.
     */
    public double getStandardMaxVelocity(double maxMotorRPM){
        if(mode != MotorMode.kServo){
            return maxMotorRPM * gearRatio.getMathematicalGearRatio() * finalDiameterMeters * Math.PI / 60.0;
        }
        return (maxMotorRPM * gearRatio.getMathematicalGearRatio()) / 60.0;
    }
/**
 * @deprecated Unit conversion ratio will be removed, and instead implemented on a need-based basis, rather than a general system
 * 
 * converts different rotational and linear units, principally converts rotations -> radians and meters -> radians
 * 
 */
    @Deprecated(since="1/7/2025", forRemoval = true)
    public double getStandardUnitConversionRatio() {
        if (mode != MotorMode.kServo) {
            return gearRatio.getMathematicalGearRatio() * finalDiameterMeters * Math.PI;
        }
        // converts to radians
        return gearRatio.getMathematicalGearRatio() * 2 * Math.PI;
    }

}