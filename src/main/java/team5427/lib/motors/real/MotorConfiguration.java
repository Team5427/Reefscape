package team5427.lib.motors.real;

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
     */
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
    
    /**
     * @return Radians Per Second
     */
    public double getStandardMaxVelocity(double maxMotorRPM) {
        return (maxMotorRPM * getStandardUnitConversionRatio()) / 60.0;
    }

    public double getStandardUnitConversionRatio() {
        if (mode != MotorMode.kServo) {
            return gearRatio.getMathematicalGearRatio() * finalDiameterMeters * Math.PI;
        }
        // converts to radians
        return gearRatio.getMathematicalGearRatio() * 2 * Math.PI;
    }

}