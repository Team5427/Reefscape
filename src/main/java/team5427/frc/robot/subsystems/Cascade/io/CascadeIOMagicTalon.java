package team5427.frc.robot.subsystems.Cascade.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.MotorConfiguration;
import team5427.lib.motors.real.SteelTalonFX;
import team5427.lib.motors.real.SteelTalonFX;

public class CascadeIOMagicTalon implements CascadeIO {

    private MagicSteelTalonFX cascadeMotorMaster;
    private MagicSteelTalonFX cascadeMotorSlave;
    private MotorConfiguration cascadeConfig;

    private MagicSteelTalonFX pivotMotorMaster;
    private MagicSteelTalonFX pivotMotorSlave;
    private MotorConfiguration pivotConfig;
    private CANcoder pivotCANcoder;

    public boolean cascadeMotorsStopped;
    public boolean pivotMotorsStopped;

    public CascadeIOMagicTalon() {
        cascadeMotorMaster = new MagicSteelTalonFX(CascadeConstants.kCascadeMasterId);
        cascadeMotorSlave = new MagicSteelTalonFX(CascadeConstants.kCascadeSlaveId);
        cascadeMotorMaster.apply(CascadeConstants.kCascadeDriverConfiguration);
        MotorConfiguration cascadeSlaveConfig = new MotorConfiguration(CascadeConstants.kCascadeDriverConfiguration);
        cascadeSlaveConfig.isInverted = true;
        cascadeMotorSlave.apply(cascadeSlaveConfig);
        pivotMotorMaster = new MagicSteelTalonFX(CascadeConstants.kPivotMasterId);
        pivotMotorSlave = new MagicSteelTalonFX(CascadeConstants.kPivotSlaveId);
        pivotMotorMaster.apply(CascadeConstants.kPivotConfiguration);

        MotorConfiguration pivotSlaveConfig = new MotorConfiguration(CascadeConstants.kPivotConfiguration);
        pivotSlaveConfig.isInverted = true;
        pivotMotorSlave.apply(pivotSlaveConfig);
        pivotCANcoder = new CANcoder(CascadeConstants.kPivotCANcoderId.getDeviceNumber());
        pivotCANcoder.getConfigurator().apply(CascadeConstants.kPivotEncoderConfig);
    }

    @Override
    public void updateInputs(CascadeIOInputs inputs) {
        inputs.velocity = MetersPerSecond.of(cascadeMotorMaster.getEncoderVelocity());
        inputs.velocityRotations = cascadeMotorMaster.getTalonFX().getVelocity().getValue();
        inputs.cascadeHeightMeters = Meters.of(cascadeMotorMaster.getEncoderPosition());
        inputs.acceleration = MetersPerSecondPerSecond
                .of(cascadeMotorMaster.getTalonFX().getAcceleration().getValue().in(RotationsPerSecondPerSecond)
                        * Math.PI * cascadeConfig.finalDiameterMeters);

        inputs.cascadeMasterMotorCurrent = cascadeMotorMaster.getTalonFX().getSupplyCurrent().getValue();
        inputs.cascadeMasterMotorVoltage = cascadeMotorMaster.getTalonFX().getSupplyVoltage().getValue();

        inputs.cascadeSlaveMotorCurrent = cascadeMotorSlave.getTalonFX().getSupplyCurrent().getValue();
        inputs.cascadeSlaveMotorVoltage = cascadeMotorSlave.getTalonFX().getSupplyVoltage().getValue();

        inputs.pivotRotation = new Rotation2d(pivotCANcoder.getAbsolutePosition().getValue());
        inputs.pivotRotationVelocity = pivotCANcoder.getVelocity().getValue();
    }

    @Override
    public void setCascadeSetpoint(Distance setpoint) {
        // cascade motor must calculate the kG outside of the motor controller as it is
        // on a pivoting arm
        if (cascadeMotorsStopped) {
            cascadeMotorMaster.getTalonFX().set(0);
            return;
        }
        // cascadeMotorMaster.getMotorConfiguration().kFF = CascadeConstants.kCascadeDriverConfiguration.isArm
        //         ? CascadeConstants.kCascadeDriverGravityFF
        //                 * pivotCANcoder.getAbsolutePosition().getValue().in(Rotations)
        //         : CascadeConstants.kCascadeDriverGravityFF;
        cascadeMotorMaster.setSetpoint(setpoint.in(Meters));
    }

    @Override
    public void setCascadeEncoderPosition(Distance setpoint) {
        cascadeMotorMaster.setEncoderPosition(setpoint.magnitude());
    }

    @Override
    public void setPivotSetpoint(Rotation2d setpoint) {
        if (pivotMotorsStopped) {
            pivotMotorMaster.getTalonFX().set(0);
            return;
        }
        pivotMotorMaster.setSetpoint(setpoint);
    }
    @Override
    public void setCANCoderPosition(Rotation2d angle) {
        pivotCANcoder.setPosition(angle.getMeasure());
    }

    @Override
    public void stopCascadeMotors(boolean stopped) {
        this.cascadeMotorsStopped = stopped;
    }

    @Override
    public void stopPivotMotors(boolean stopped) {
        this.pivotMotorsStopped = stopped;
    }

}