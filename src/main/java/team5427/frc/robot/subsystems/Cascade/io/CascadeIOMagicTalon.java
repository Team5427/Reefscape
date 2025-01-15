package team5427.frc.robot.subsystems.Cascade.io;

import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.MotorConfiguration;
import team5427.lib.motors.real.SteelTalonFX;

public class CascadeIOMagicTalon implements CascadeIO{
    private MagicSteelTalonFX cascadeMotorMaster;
    private MagicSteelTalonFX cascadeMotorSlave;
    private MotorConfiguration config;
    private double cascadeSetpointMeters;


    public CascadeIOMagicTalon(){
        cascadeMotorMaster = new MagicSteelTalonFX(null);
    }

    @Override
    public void updateInputs(CascadeIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setCascadeSetpoint(double meters) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCascadeSetpoint'");
    }
    
}
