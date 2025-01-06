package team5427.lib.motors.simulated;

import com.ctre.phoenix6.sim.ChassisReference;

import team5427.lib.drivers.ComplexGearRatio;

public class SimMotorConfiguration {

    public static enum SimMotorMode {
        kFlywheel,
        kServo,
        kLinear
    }

    public ChassisReference orientation;

    public double kP, kI, kD, kFF;
    public double kS, kV, kA;

    public ComplexGearRatio gearRatio;
    public double finalDiameterMeters;
    public double unitConversionRatio;
    public double maxVelocity, maxAcceleration;

    public SimMotorMode mode;

    public SimMotorConfiguration() {
        orientation = ChassisReference.Clockwise_Positive;

        kP = kI = kD = kFF = 0.0;
        kS = kV = kA = 0.0;

        gearRatio = new ComplexGearRatio();
        finalDiameterMeters = 1.0;
        unitConversionRatio = 1.0;
        maxVelocity = 1.0;
        maxAcceleration = 1.0;
        mode = SimMotorMode.kFlywheel;
    }

    public SimMotorConfiguration(SimMotorConfiguration parent) {
        orientation = parent.orientation;

        kP = parent.kP;
        kI = parent.kI;
        kD = parent.kD;
        kFF = parent.kFF;

        kS = parent.kS;
        kV = parent.kV;
        kA = parent.kA;

        gearRatio = parent.gearRatio;
        unitConversionRatio = parent.unitConversionRatio;
        finalDiameterMeters = parent.finalDiameterMeters;

        maxVelocity = parent.maxVelocity;
        maxAcceleration = parent.maxAcceleration;
        mode = parent.mode;
    }

    public double getStandardMaxVelocity(double maxMotorRPM) {
        return maxMotorRPM * getStandardUnitConversionRatio();
    }

    public double getStandardUnitConversionRatio() {
        if (mode != SimMotorMode.kServo)
            return gearRatio.getMathematicalGearRatio() * finalDiameterMeters * Math.PI;
        return gearRatio.getMathematicalGearRatio() * 2 * Math.PI;
    }

}