package team5427.frc.robot.subsystems.Cascade.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import team5427.frc.robot.Constants.CascadeConstants;

public class CascadeIOSim implements CascadeIO {

    private DCMotorSim cascadeMotors;
    private Distance cascadeSetpointMeters;

    private DCMotorSim pivotMotors;
    private Rotation2d pivotSetpoint;

    private static final double cascadeMotorInertia = 0.05;
    private static final double pivotMotorInertia = 0.03;

    public CascadeIOSim() {
        cascadeMotors = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        CascadeConstants.kCascadeDriverConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2)
                                : DCMotor.getKrakenX60(2),
                        cascadeMotorInertia,
                        CascadeConstants.kCascadeDriverConfiguration.gearRatio.getMathematicalGearRatio()),
                CascadeConstants.kCascadeDriverConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2)
                        : DCMotor.getKrakenX60(2));

        pivotMotors = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        CascadeConstants.kPivotConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2)
                                : DCMotor.getKrakenX60(2),
                        pivotMotorInertia,
                        CascadeConstants.kPivotConfiguration.gearRatio.getMathematicalGearRatio()),
                CascadeConstants.kPivotConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2) : DCMotor.getKrakenX60(2));
    }

    @Override
    public void updateInputs(CascadeIOInputs inputs) {
        inputs.acceleration = LinearAcceleration.ofBaseUnits(
                cascadeMotors.getAngularAcceleration().in(RotationsPerSecondPerSecond)
                        * CascadeConstants.kCascadeDriverConfiguration.finalDiameterMeters * Math.PI,
                MetersPerSecondPerSecond);

        inputs.velocity = LinearVelocity.ofBaseUnits(
                cascadeMotors.getAngularVelocity().in(RotationsPerSecond)
                        * CascadeConstants.kCascadeDriverConfiguration.finalDiameterMeters * Math.PI,
                MetersPerSecond);

        inputs.velocityRotations = cascadeMotors.getAngularVelocity();

        inputs.cascadeHeightMeters = Distance.ofBaseUnits(
                cascadeMotors.getAngularPosition().in(Rotations)
                        * CascadeConstants.kCascadeDriverConfiguration.finalDiameterMeters * Math.PI,
                Meters);

        inputs.cascadeMasterMotorCurrent = Amps.of(cascadeMotors.getCurrentDrawAmps());
        inputs.cascadeSlaveMotorCurrent = Amps.of(cascadeMotors.getCurrentDrawAmps());

        inputs.cascadeMasterMotorVoltage = Volts.of(cascadeMotors.getInputVoltage());
        inputs.cascadeSlaveMotorVoltage = Volts.of(cascadeMotors.getInputVoltage());

        inputs.pivotRotation = Rotation2d.fromRotations(pivotMotors.getAngularPosition().in(Rotations));
        inputs.pivotRotationVelocity = pivotMotors.getAngularVelocity();
        inputs.pivotRotationAcceleration = pivotMotors.getAngularAcceleration();

        inputs.pivotMasterMotorCurrent = Amps.of(pivotMotors.getCurrentDrawAmps());
        inputs.pivotSlaveMotorCurrent = Amps.of(pivotMotors.getCurrentDrawAmps());

        inputs.pivotMasterMotorVoltage = Volts.of(pivotMotors.getInputVoltage());
        inputs.pivotSlaveMotorVoltage = Volts.of(pivotMotors.getInputVoltage());
    }

    @Override
    public void setCascadeSetpoint(Distance setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCascadeSetpoint'");
    }

    @Override
    public void setCascadeEncoderPosition(Distance setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCascadeEncoderPosition'");
    }

    @Override
    public void setPivotSetpoint(Rotation2d setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPivotSetpoint'");
    }

    @Override
    public void setCANCoderPosition(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCANCoderPosition'");
    }

}