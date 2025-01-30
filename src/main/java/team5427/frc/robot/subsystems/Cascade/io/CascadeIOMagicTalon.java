package team5427.frc.robot.subsystems.Cascade.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
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

public class CascadeIOMagicTalon implements CascadeIO{

    private MagicSteelTalonFX cascadeMotorMaster;
    private MagicSteelTalonFX cascadeMotorSlave;
    private MotorConfiguration cascadeConfig;

    private MagicSteelTalonFX pivotMotorMaster;
    private MagicSteelTalonFX pivotMotorSlave;
    private MotorConfiguration pivotConfig;

    private CANcoder pivotCANcoder;

    public CascadeIOMagicTalon(){
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setCANCoderPosition(Rotation2d angle) {
        pivotCANcoder.setPosition(angle.getMeasure());
    }

    
}
