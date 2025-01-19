package team5427.frc.robot.subsystems.EndEffector.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.SteelTalonFX;

public class EndEffectorIOTalon implements EndEffectorIO {
    private MagicSteelTalonFX pivotMotor;
    private MagicSteelTalonFX wristMotor;
    private SteelTalonFX coralRollerMotor;
    private SteelTalonFX algaeRollerMotor;

    public EndEffectorIOTalon(){
        
    }


    @Override
    public void setCoralRollerSetpoint(AngularVelocity velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCoralRollerSetpoint'");
    }

    @Override
    public void setCoralWristSetpoint(Rotation2d setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCoralWristSetpoint'");
    }

    @Override
    public void setAlgaeRollerSetpoint(AngularVelocity velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAlgaeRollerSetpoint'");
    }

    @Override
    public void setPivotSetpoint(Rotation2d setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPivotSetpoint'");
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        
    }
    
}
